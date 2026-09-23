#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found: $ros_setup (set ROS_SETUP to override)" >&2
  exit 1
fi
set +u
source "$ros_setup"
set -u
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$script_dir/make_camera_path_from_bag.py" "$@"
