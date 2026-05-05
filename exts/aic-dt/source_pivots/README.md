# Source pivots — Plan 04-03 host-build path

These files override the corresponding paths in `~/Documents/aic/` when
`build_aic_engine_host.sh` vendors them into `~/aic_humble_ws/src/`.

Filename convention: path separators are encoded as `@`. For example,
`aic_scoring@src@ScoringTier2.cc` overlays at `aic_scoring/src/ScoringTier2.cc`.

## Why these patches exist

The AIC repo (`~/Documents/aic/`) is authored against ROS 2 **kilted** + g++-13.
This project's host is Ubuntu 22.04 with **humble** + g++-11. The Plan 04-03
host-build pivot resolves the kilted↔humble fastrtps RMW interop blocker
by compiling `aic_engine` + `aic_adapter` as native humble processes
(no Docker, no cross-distro DDS bridge). These patches make the kilted-era
sources compile against humble:

| File | Patch |
|------|-------|
| `aic_scoring@CMakeLists.txt` | gz_math_vendor (gz-math8, kilted-only) → system gz-math7 (libgz-math7-dev) |
| `aic_scoring@src@ScoringTier2.cc` | humble `create_generic_subscription` 1-arg lambda + humble `rosbag2_cpp::Writer::write` 4-arg overload |
| `aic_adapter@CMakeLists.txt` | `find_package(fmt REQUIRED)` + link `fmt::fmt` |
| `aic_adapter@include@aic_adapter@aic_adapter.hpp` | `<format>` → `<fmt/format.h>` |
| `aic_adapter@src@aic_adapter.cpp` | `<format>` → `<fmt/format.h>` and `std::format` → `fmt::format` |

`simulation_interfaces` is sourced from the apt package
`ros-humble-simulation-interfaces`, extracted into
`~/aic_humble_ws/vendored_apt/extracted/` (no sudo required).

## Architectural fidelity

`PROJECT.md`'s zero-divergence guarantee is preserved: the patches are
build-time only (compiler / dependency-name shims) and produce **the same
ROS topic surface the original kilted aic_engine produces**. They do NOT
modify trial config, ROS topic names, scoring logic, or any user-visible
behaviour. ScoringTier2 deprecation warnings are logged but the deprecated
write() overload is API-identical.

## Re-deriving these patches

If `~/Documents/aic` is updated and these patches need to be regenerated:

```bash
# After running build_aic_engine_host.sh and verifying the build succeeds,
# copy the locally-patched files back into source_pivots/ for VCS:
cp ~/aic_humble_ws/src/aic_scoring/CMakeLists.txt          aic_scoring@CMakeLists.txt
cp ~/aic_humble_ws/src/aic_scoring/src/ScoringTier2.cc     aic_scoring@src@ScoringTier2.cc
cp ~/aic_humble_ws/src/aic_adapter/CMakeLists.txt          aic_adapter@CMakeLists.txt
cp ~/aic_humble_ws/src/aic_adapter/include/aic_adapter/aic_adapter.hpp \
   aic_adapter@include@aic_adapter@aic_adapter.hpp
cp ~/aic_humble_ws/src/aic_adapter/src/aic_adapter.cpp     aic_adapter@src@aic_adapter.cpp
```
