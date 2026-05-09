#!/bin/bash
# Sister probe to probe_taskboard_authoring.py — drives a manual plug-port
# collision via MCP and observes /scoring/insertion_event.
#
# Implements taskboard-prim-authoring acceptance gate 2 (plans/prd.json):
#   /scoring/insertion_event fires when a plug contacts its target port
#   (verified by ros2 topic echo /scoring/insertion_event during a manual
#   plug-port collision via MCP).
#
# Implementation lives in the matching .py for testability and ease of debug;
# this wrapper sources ~/env_isaaclab/bin/activate (Python 3.11 + rclpy +
# humble msgs) and sets ROS_DOMAIN_ID=7 (sim isolation per CLAUDE.md).
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1; pwd)

# The python helper spawns its own zenoh-aware subscriber subprocess. Here we
# only need a Python with the env_isaaclab venv (has the MCP socket helpers).
# Sourcing /opt/ros + activate chain references COLCON_CURRENT_PREFIX with
# `set -u` semantics — sidestep by disabling -u just for the activate.
set +u
# shellcheck disable=SC1090
source ~/env_isaaclab/bin/activate
set -u

python3 "${SCRIPT_DIR}/probe_insertion_event_fires.py" "$@"
