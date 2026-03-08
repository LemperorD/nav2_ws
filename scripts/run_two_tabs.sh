#!/usr/bin/env bash
# Run behavior and navigation in two gnome-terminal tabs
# Usage: ./run_two_tabs.sh [behavior_name]

set -euo pipefail

BEHAVIOR=${1:-decision_simple}

# Ensure script is run from repo root or adjust paths
WORKDIR="$HOME/nav2_ws"

BEHAVIOR_CMD="cd $WORKDIR && source install/setup.bash && ./scripts/floor10_nav.sh behavior ${BEHAVIOR}; exec bash"
NAV_CMD="cd $WORKDIR && source install/setup.bash && ./scripts/floor10_nav.sh nav; exec bash"

gnome-terminal \
# Try to open two tabs; as fallback open two separate windows which works reliably
# Some gnome-terminal builds/platforms may not create multiple tabs from a single call,
# so open two windows to ensure both commands appear.
gnome-terminal --window --title="behavior" -- bash -lc "$BEHAVIOR_CMD" &
gnome-terminal --window --title="navigation" -- bash -lc "$NAV_CMD" &
