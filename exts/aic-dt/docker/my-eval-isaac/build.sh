#!/bin/bash
# Reference: builds my-eval-isaac:v1 from the local Dockerfile.
# Plan 04-03 Task 1.
set -euo pipefail
cd "$(dirname "$0")"

echo "[build] building my-eval-isaac:v1 from $(pwd)"
docker build -t my-eval-isaac:v1 .

echo ""
echo "[build] complete. Test with:"
echo ""
echo "  docker run --rm --net=host \\"
echo "    -e ROS_DOMAIN_ID=7 \\"
echo "    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \\"
echo "    -e CONFIG_PATH=/ws_aic/install/share/aic_engine/config/sample_config.yaml \\"
echo "    my-eval-isaac:v1"
echo ""
echo "Or via the wrapper:"
echo ""
echo "  bash ../scripts/run_aic_engine_against_isaac_sim.sh trial_1"
