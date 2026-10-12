#!/usr/bin/env python3
"""Restore the pinned ikd-Tree commit using a small, reviewed local delta bundle."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CHILD = ROOT / 'include/ikd-Tree'
TARGET = '5c5624219575f88c67da8b77bdeb98c302ce8dec'
BUNDLE = ROOT / 'dependencies/ikd_tree_startup.bundle'


def git(directory, *arguments):
    return subprocess.check_output(
        ['git', '-c', 'core.hooksPath=/dev/null', '-C', str(directory), *arguments],
        text=True,
    ).strip()


def main():
    git(ROOT, 'submodule', 'init')
    created = not (CHILD / '.git').exists()
    if created:
        url = git(ROOT, 'config', '--file', '.gitmodules', '--get',
                  'submodule.include/ikd-Tree.url')
        subprocess.run(['git', 'clone', '--no-checkout', '--', url, str(CHILD)], check=True)
    if not created and git(CHILD, 'status', '--porcelain'):
        raise SystemExit(f'{CHILD}: preserve local changes before updating the submodule')
    # The prerequisite is the upstream e2e3f4e commit. No binary/build artifacts
    # or unrelated history are included in this approximately 1 KiB bundle.
    git(CHILD, 'bundle', 'verify', str(BUNDLE))
    git(CHILD, 'fetch', '--no-recurse-submodules', '--no-tags', str(BUNDLE), 'refs/remotes/transfer/avia-ground-start')
    git(CHILD, 'checkout', '--detach', TARGET)
    git(ROOT, 'submodule', 'absorbgitdirs', 'include/ikd-Tree')
    print(f'ikd-Tree ready: {TARGET}')


if __name__ == '__main__':
    main()
