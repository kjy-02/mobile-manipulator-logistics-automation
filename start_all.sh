#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BASE_SETUP="${SCOUT_BASE_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
if [[ ! -f "$BASE_SETUP" ]]; then
  echo "Dependency workspace setup not found: $BASE_SETUP" >&2
  echo "Set SCOUT_BASE_SETUP to the built dependency workspace devel/setup.bash." >&2
  exit 2
fi
source "$BASE_SETUP"
# Incremental build includes edits to the local C++/Python/config copies.
catkin_make -C "$SCRIPT_DIR/local_ws" -j2 -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$SCRIPT_DIR/local_ws/devel/setup.bash"
exec python3 "$SCRIPT_DIR/start_all.py" "$@"
