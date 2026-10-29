#!/usr/bin/env bash
set -euo pipefail

upstream_dir="${1:-/home/rilab/catkin_ws/src/dwa_planner}"
expected_revision="6387a9660c2064fa0545ac81d55f4bbe7962651f"

actual_revision="$(git -C "$upstream_dir" rev-parse HEAD)"
if [[ "$actual_revision" != "$expected_revision" ]]; then
  echo "revision mismatch: expected $expected_revision, got $actual_revision" >&2
  exit 1
fi

if [[ -n "$(git -C "$upstream_dir" status --porcelain=v1)" ]]; then
  echo "upstream checkout is not clean" >&2
  exit 1
fi

cd "$upstream_dir"
sha256sum --check <<'HASHES'
1b3a9ff7f391bf5ca659f41292757f356d2e54b3287303b08f40ddba3eeef6b4  include/dwa_planner/dwa_planner.h
255376d98501c23c346d43981ff1e17192b3a6e4def56ab138047b6894d2aedd  src/dwa_planner.cpp
5e1c8f757ccf0c7bb4a8db63ad18693a75bfb21eccddf491b00b5c71a6fabe3c  src/dwa_planner_node.cpp
b8bc61533774c77ba019357d5a6d8e147a122f356b25d0be8f07a2cc5b6f193c  src/parameters.cpp
ed7ad276cd49cdd0aac446e3039976d0847621570ade6811146c71fc7b574c01  CMakeLists.txt
a13f0da34659e5f7f7c6e1bd88184052ad507d5d7accc00fd86d79ceb8a56edc  package.xml
386341bdb78e715920f244d4d8c229b3e75721431833a64f954d6f8b41a580d5  config/dwa_param.yaml
42bcd6c66d06cd69b46311d10a809d09af29aea2b4bbf3905f9d215db0d1e912  config/robot_param.yaml
54c6dde8b68d5b74e6ac00116c61127a0778acbd1149b6e662d8efe2ec73195c  LICENSE
HASHES

echo "upstream dwa_planner verified at $actual_revision"
