# Internal FAST-LIO prediction contract, version 1. State pose is world<-IMU.
# Gravity uses the bundled MTK S2<double,98090,10000,1> convention.
uint32 schema_version
string session_id
uint64 epoch
uint64 snapshot_sequence
uint64 correction_sequence
std_msgs/Header header
int64 last_correction_stamp_ns
int64 release_stamp_ns
string phase
# Existing mapper phase, forwarded atomically with the selected pose.
string localization_phase
bool initialized
bool gyro_bias_frozen
bool recovery_requires_restart
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 velocity_world
geometry_msgs/Vector3 gyro_bias
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gravity
geometry_msgs/Quaternion lidar_to_imu_rotation
geometry_msgs/Vector3 lidar_to_imu_translation
# Row-major covariance, error order: p,rot,extrR,extrT,v,bg,ba,grav(S2).
float64[529] covariance
# Row-major matrix actually used by IKFoM, order gyro,accel,bg,ba.
float64[144] process_noise
# Apply to raw acceleration before subtracting accel_bias, exactly once.
float64 acceleration_scale
# corrected/source time = raw IMU header stamp + this offset.
int64 imu_time_offset_ns
uint32 effective_measurement_count
float64 residual_mean_m
