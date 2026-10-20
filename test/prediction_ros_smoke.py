#!/usr/bin/env python3
"""Exercise the real predictor executables with analytical ROS inputs, without a simulator.

Source the ROS/FAST-LIO overlays and start the configured middleware router first.
This checks message interfaces, parameter types, source grids, correction replay,
disabled behavior and absence of publications while IMU input is stopped.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from fast_lio.msg import FilterState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def stamp(message, nanoseconds):
    message.sec, message.nanosec = divmod(nanoseconds, 10**9)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=6)
    args = parser.parse_args()
    if args.seconds < 3:
        parser.error('--seconds must be at least three')
    args.output.mkdir(parents=True, exist_ok=False)
    binary = Path(get_package_prefix('fast_lio')) / 'lib/fast_lio/fastlio_imu_prediction'
    rclpy.init()
    node = Node('fastlio_prediction_smoke_source')
    qos = QoSProfile(depth=2048, reliability=ReliabilityPolicy.RELIABLE)
    state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    imu_pub = node.create_publisher(Imu, '/prediction_smoke/imu', qos)
    state_pub = node.create_publisher(FilterState, '/prediction_smoke/state', state_qos)
    children = []
    subscriptions = []
    outputs = {rate: [] for rate in (0, 20, 30, 40)}
    statuses = {rate: [] for rate in outputs}
    report = dict(scope='Analytical ROS fixture; no flight or simulator, no driver changes',
                  passed=False, rates={})
    stream = (args.output / 'outputs.jsonl').open('x', buffering=1)
    try:
        for rate in outputs:
            topic = f'/prediction_smoke/odom{rate}'
            status_topic = f'/prediction_smoke/status{rate}'

            def output(message, rate=rate):
                source_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                row = dict(rate=rate, stamp_ns=source_ns, monotonic_ns=time.monotonic_ns(),
                           position=[message.pose.pose.position.x, message.pose.pose.position.y,
                                     message.pose.pose.position.z],
                           pose_covariance=list(message.pose.covariance),
                           twist_covariance=list(message.twist.covariance))
                outputs[rate].append(row)
                stream.write(json.dumps(row, allow_nan=False) + '\n')

            def status(message, rate=rate):
                for item in message.status:
                    if item.name == 'fastlio_prediction':
                        statuses[rate].append({value.key: value.value for value in item.values})

            subscriptions.append(node.create_subscription(Odometry, topic, output, qos))
            subscriptions.append(node.create_subscription(DiagnosticArray, status_topic, status, qos))
            arguments = [str(binary), '--ros-args', '-r', f'__node:=prediction_smoke_{rate}',
                         '-p', 'imu_topic:=/prediction_smoke/imu',
                         '-p', 'state_topic:=/prediction_smoke/state',
                         '-p', f'output_topic:={topic}', '-p', f'status_topic:={status_topic}',
                         '-p', f'prediction.output_rate_hz:={float(rate or 30)}']
            if rate:
                arguments.extend(['-p', 'prediction.enabled:=true'])
            with (args.output / f'predictor-{rate}.log').open('x') as log:
                children.append(subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 15
        while imu_pub.get_subscription_count() < 3 or state_pub.get_subscription_count() < 3:
            rclpy.spin_once(node, timeout_sec=0.01)
            if any(child.poll() is not None for child in children):
                raise RuntimeError('Predictor startup failed; inspect node logs')
            if time.monotonic() > deadline:
                raise TimeoutError('Predictors did not subscribe to both source topics')

        origin_ns = time.time_ns()
        origin_wall = time.monotonic()
        report['origin_ns'] = origin_ns
        for index in range(args.seconds * 200 + 1):
            deadline = origin_wall + index / 200
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=max(0.0, min(0.001, deadline - time.monotonic())))
            source_ns = origin_ns + index * 5_000_000
            imu = Imu()
            stamp(imu.header.stamp, source_ns)
            imu.header.frame_id = 'imu'
            imu.linear_acceleration.z = 9.809
            for axis in (0, 4, 8):
                imu.angular_velocity_covariance[axis] = 1e-4
            imu_pub.publish(imu)
            if index % 20 == 0:
                state = FilterState()
                state.schema_version = 1
                state.session_id = 'analytical-ros-smoke'
                state.epoch = 1
                state.snapshot_sequence = index // 20 + 1
                state.correction_sequence = state.snapshot_sequence
                stamp(state.header.stamp, source_ns)
                state.header.frame_id = 'camera_init'
                state.last_correction_stamp_ns = source_ns
                state.release_stamp_ns = origin_ns
                state.phase = 'LIDAR_TRACKING'
                state.initialized = True
                state.orientation.w = state.lidar_to_imu_rotation.w = 1.0
                state.gravity.z = -9.809
                state.velocity_world.x, state.velocity_world.y = 1.0, -0.5
                state.position.x, state.position.y = index / 200, -index / 400
                state.acceleration_scale = 1.0
                state.effective_measurement_count = 100
                for diagonal in range(23):
                    state.covariance[diagonal * 24] = 1e-3
                for diagonal in range(12):
                    state.process_noise[diagonal * 13] = 1e-4
                state_pub.publish(state)
            rclpy.spin_once(node, timeout_sec=0)
        # Drain transport before testing that an idle input generates no new poses.
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
        before_idle = {rate: len(rows) for rate, rows in outputs.items()}
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
        for rate in (20, 30, 40):
            rows = outputs[rate]
            actual = [row['stamp_ns'] for row in rows]
            expected = [origin_ns + (index * 10**9 + rate // 2) // rate
                        for index in range(1, args.seconds * rate + 1)]
            errors = [math.dist(row['position'], [(row['stamp_ns'] - origin_ns) * 1e-9,
                                                  -(row['stamp_ns'] - origin_ns) * 0.5e-9, 0])
                      for row in rows]
            covariance_ok = all(all(math.isfinite(value) for value in row[key]) and
                                all(row[key][index * 7] >= 0 for index in range(6))
                                for row in rows for key in ('pose_covariance', 'twist_covariance'))
            last_status = statuses[rate][-1] if statuses[rate] else {}
            checks = dict(exact_source_grid=actual == expected,
                          analytical_trajectory=bool(errors) and max(errors) < 1e-6,
                          covariance_finite_nonnegative=covariance_ok,
                          no_publication_without_imu=len(rows) == before_idle[rate],
                          stale_status=last_status.get('valid') == 'false',
                          process_alive=children[(20, 30, 40).index(rate) + 1].poll() is None)
            report['rates'][str(rate)] = dict(count=len(rows), expected_count=len(expected),
                maximum_position_error_m=max(errors) if errors else None,
                checks=checks, passed=all(checks.values()))
        report['disabled_no_output'] = not outputs[0]
        report['passed'] = report['disabled_no_output'] and all(
            item['passed'] for item in report['rates'].values())
        return 0 if report['passed'] else 1
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
