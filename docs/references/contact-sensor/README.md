# Contact-sensor reference material

Three NVIDIA source files that were instrumental in diagnosing the Isaac
Sim 5.0 `ContactSensor`-wrapper failure and choosing the direct
`omni.physx` contact-report subscription path used in
`exts/soarm101-dt/so_arm101_dt/extension.py::_setup_contact_sensors`.

Kept here (outside `tmp/`, which is `.gitignore`d) so future debugging
sessions can read them without re-cloning the 1+ GB IsaacSim repo.

## Files

| File | Origin | Why it matters |
|---|---|---|
| `nvidia_example_contact_sensor.py` | `IsaacSim/source/extensions/isaacsim.sensors.physics.examples/.../contact_sensor.py` | NVIDIA's own working example showing the raw `IsaacSensorCreateContactSensor` kit-command pattern — the pattern we tried first and that still returned `is_valid=False` in 5.0. Confirmed the wrapper wasn't the bug. |
| `nvidia_wrapper_contact_sensor.py` | `IsaacSim/source/extensions/isaacsim.sensors.physics/python/impl/contact_sensor.py` | The Python `ContactSensor` wrapper. Its `__init__` has an "already-exists" branch (lines 40–50) that skips `IsaacSensorCreateContactSensor` when the sensor prim already exists — that's the hot-reload failure mode we hit. |
| `nvidia_wrapper_commands.py` | `IsaacSim/source/extensions/isaacsim.sensors.physics/python/impl/commands.py` | `IsaacSensorCreateContactSensor` command source. Uses `get_next_free_path()` which silently appends `_1`, `_2` on collision — explains why the *actual* created path can differ from the `path=` argument. |

## Why we didn't end up using the wrapper

See `docs/DEBUG-GUIDE.md` § 8.5b. Short version: in Isaac Sim 5.0 the
`omni.physx.contact` extension that the wrapper's C++ backend depends on
is not present under that name, so every sensor came back
`is_valid=False`. We subscribe to
`omni.physx.get_physx_simulation_interface().subscribe_contact_report_events`
directly and filter on `PhysxContactReportAPI`-tagged prims — same
underlying mechanism, no missing extension.

## Origin / attribution

NVIDIA Isaac Sim, Apache 2.0 licensed. Full clones available on demand
in `tmp/contact_ref_IsaacSim/` (re-cloneable via
`git clone https://github.com/isaac-sim/IsaacSim.git`).
