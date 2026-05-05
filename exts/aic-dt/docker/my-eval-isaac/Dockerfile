# Reference: 04-RESEARCH.md Q4 Option 4B (derived image w/ engine-only entrypoint).
# Reference: 04-01-SUMMARY.md A2=PASS (RMW interop confirmed; no humble-base rebuild needed).
# Reference: ~/Documents/aic/docker/aic_eval/Dockerfile (workspace at /ws_aic/install).
#
# Derives from the official aic_eval (kilted) image to reuse aic_engine +
# aic_adapter + aic_description installed packages. Overrides the upstream
# entrypoint (which forces zenoh + aic_gz_bringup.launch.py — Gazebo + ros2_control
# + bridge spawners we explicitly do NOT want when Isaac Sim is the source of truth).
#
# Plan 04-03 Sub-A (Wave 1 A2=PASS): Option A (run via stock aic_eval w/ entrypoint
# override at docker run time) is also viable. This derived image is the
# preferred steady-state form (one less flag set per invocation) and tracks the
# entrypoint contract in source instead of in the wrapper script.

FROM ghcr.io/intrinsic-dev/aic/aic_eval:latest

# Talk to Isaac Sim's bridge on the host's domain 7 with vanilla DDS.
# Override upstream's RMW_IMPLEMENTATION=rmw_zenoh_cpp.
ENV ROS_DOMAIN_ID=7
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Disable upstream's ZENOH router-check loop — we don't run zenohd.
ENV ZENOH_ROUTER_CHECK_ATTEMPTS=0

COPY entrypoint.sh /entrypoint-isaac.sh
RUN chmod +x /entrypoint-isaac.sh

# Replace upstream entrypoint (which runs aic_gz_bringup.launch.py wholesale).
ENTRYPOINT ["/entrypoint-isaac.sh"]
CMD []
