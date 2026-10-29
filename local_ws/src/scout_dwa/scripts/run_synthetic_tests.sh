#!/usr/bin/env bash
set -euo pipefail
workspace="${1:-/home/rilab/catkin_ws}"
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
export ROS_HOME
ROS_HOME="$(mktemp -d /tmp/scout_dwa_rostest.XXXXXX)"
"$workspace/src/scout_dwa/scripts/verify_upstream.sh"
python3 "$workspace/src/scout_dwa/scripts/validate_integration_config.py" \
  "$workspace/src/scout_dwa/config/synthetic_dwa.yaml" \
  "$workspace/src/scout_dwa/config/cloud_adapter_reference.yaml"
timeout --signal=TERM 300 cmake --build "$workspace/build" --target run_tests_scout_dwa -j2
catkin_test_results "$workspace/build/test_results/scout_dwa"
