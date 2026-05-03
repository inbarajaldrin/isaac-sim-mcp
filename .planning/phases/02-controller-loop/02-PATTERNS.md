# Phase 2: Controller Loop - Pattern Map

**Mapped:** 2026-05-03
**Files analyzed:** 6 (4 NEW + 1 MODIFY + 1 OPTIONAL helper script)
**Analogs found:** 6 / 6 (every Phase 2 file has a battle-tested Phase 1 analog — Phase 2 is structurally an inversion of Phase 1)

## Inversion principle

Phase 2 = Phase 1 inverted on the rclpy axis:

| Axis | Phase 1 (`parity_publishers.py`) | Phase 2 (`controller_loop.py`) |
|------|----------------------------------|---------------------------------|
| rclpy direction | publishers (sim → ROS) | subscribers (ROS → sim) AND publishers (sim → ROS for state + contacts) |
| Control flow | read articulation → publish | spin_once → apply to articulation → publish state |
| Message ABI | stock `sensor_msgs`, `tf2_msgs`, `geometry_msgs` (already in 3.11 workspace) | **custom** `aic_control_interfaces` + `ros_gz_interfaces` (REQUIRES workspace rebuild — D-05 landmine) |
| Side channels | none | `omni.physx` contact-report subscription (separate path that batches into the per-tick publish) |
| Lifecycle | `start()`/`stop()` driven by `_start_aic_parity_publishers()` helper | identical pattern, mirror as `_start_aic_controller_loop()` |
| Path discipline | Python 3.11 sys.path swap + sys.modules eviction (lines 32-124) | **identical, copy verbatim** (custom messages need same ABI rules) |

The single LOC structural change beyond the inversion: `controller_loop.py` adds an `ArticulationKinematicsSolver` field for IK/FK and an `omni.physx` contact-report subscription handle.

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match Quality |
|-------------------|------|-----------|----------------|---------------|
| `exts/aic-dt/aic_dt/controller_loop.py` (NEW) | rclpy class on physics-tick | bidirectional (2 sub + 2 pub) + IK/FK + contact-event | `exts/aic-dt/aic_dt/parity_publishers.py` | **exact (inverted mirror)** |
| `exts/aic-dt/aic_dt/extension.py` (MODIFY) | MCP atom integration (4-surface contract × 2 atoms + on_shutdown + quick_start) | request-response (handler) + lifecycle | Phase 1's `setup_tf_publisher` + `setup_joint_state_publisher` additions (extension.py lines 197-204, 358-359, 1473-1559, 2730-2744, 3186-3192) | exact |
| `exts/aic-dt/scripts/smoke_test_aic_controller.py` (NEW) | external Python 3.10 verification script | request-response (publish cmds, subscribe state, assert) | `exts/aic-dt/scripts/smoke_test_aic_parity.py` | **exact (extends with publishers + 2 new subscribers)** |
| `exts/aic-dt/docs/aic-msgs-setup.md` (NEW) | docs (workspace rebuild instructions) | static reference | `exts/aic-dt/docs/rclpy-setup.md` | exact |
| `exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh` (NEW, helper) | bash snapshot/discovery script | one-shot capture | `exts/aic-dt/scripts/snapshot_aic_eval.sh` | exact |
| `exts/aic-dt/scripts/build_aic_msgs.sh` (NEW, helper — optional) | bash workspace build wrapper | one-shot setup | `exts/aic-dt/docs/rclpy-setup.md` build commands (no scripted analog yet) | role-match (no current bash analog) |

---

## Pattern Assignments

### `exts/aic-dt/aic_dt/controller_loop.py` (rclpy class on physics-tick, bidirectional)

**Analog:** `exts/aic-dt/aic_dt/parity_publishers.py` (529 LOC — mirror inverted)

**Module-level path discipline** (lines 32-124, COPY VERBATIM — same constraint, custom AIC messages have identical Python 3.11 ABI requirement):

```python
# parity_publishers.py:32-59 — sys.path setup at module-import time
import os
import sys

_RCLPY_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages"
_ISAAC_ROS_311 = "/home/aaugus11/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local/lib/python3.11/dist-packages"

_STALE_310_FRAGMENTS = (
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/dist-packages",
)
sys.path[:] = [p for p in sys.path if not any(s in p for s in _STALE_310_FRAGMENTS)]

for _new_path in (_ISAAC_ROS_311, _RCLPY_311):
    if os.path.isdir(_new_path) and _new_path not in sys.path:
        sys.path.insert(0, _new_path)
```

**Helpers** `_force_python311_ros_paths()` and `_ensure_rclpy_clean_import()` (parity_publishers.py:62-124) — **COPY VERBATIM**, but extend `_ROS_PREFIXES` tuple (line 112) with `"aic_control_interfaces"` and `"ros_gz_interfaces"` so the next-import resolver picks up the freshly-built 3.11 ABI versions:

```python
_ROS_PREFIXES = (
    "rclpy", "rcl_interfaces", "sensor_msgs", "tf2_msgs", "geometry_msgs",
    "std_msgs", "builtin_interfaces", "action_msgs", "lifecycle_msgs",
    # ... same as parity_publishers.py ...
    "aic_control_interfaces",  # NEW for Phase 2
    "ros_gz_interfaces",       # NEW for Phase 2
)
```

**Class lifecycle pattern** (parity_publishers.py:225-368 — class skeleton with `start()/stop()` and `_on_physics_step()`):

```python
class AicParityPublishers:
    def __init__(self, robot_xform_path: str = "/World/UR5e/aic_unified_robot"):
        self._robot_xform_path = robot_xform_path
        self._node = None
        self._js_pub = None
        self._tf_pub = None
        self._tf_static_pub = None
        self._physx_sub = None
        self._articulation = None
        # ...

    def start(self):
        self.stop()  # idempotent reset
        _force_python311_ros_paths()
        _ensure_rclpy_clean_import()
        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
            from sensor_msgs.msg import JointState
            from tf2_msgs.msg import TFMessage
        except ImportError as exc:
            print(f"[AIC-DT][parity] rclpy import failed: {exc!r}")
            print(f"[AIC-DT][parity] Ensure {_RCLPY_311} exists; see exts/aic-dt/docs/rclpy-setup.md")
            return False

        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception as exc:
                print(f"[AIC-DT][parity] rclpy.init() failed: {exc!r}")
                return False

        self._node = rclpy.create_node("aic_dt_parity_publisher")
        # ... QoS setup, publishers, articulation init ...

        try:
            from omni.physx import get_physx_interface
            self._physx_sub = get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
        except Exception as exc:
            print(f"[AIC-DT][parity] subscribe_physics_step_events failed: {exc!r}")
        return True

    def stop(self):
        # idempotent teardown — physx_sub release, node.destroy_node(), null all attrs
        ...

    def _on_physics_step(self, dt: float):
        if self._node is None:
            return
        try:
            now = self._node.get_clock().now().to_msg()
        except Exception:
            return
        try:
            self._publish_joint_state(now)
            self._publish_tf_dynamic(now)
            if not self._tf_static_published:
                self._publish_tf_static(now)
                self._tf_static_published = True
        except Exception as exc:
            if not getattr(self, "_logged_publish_error", False):
                print(f"[AIC-DT][parity] publish error: {exc!r}")
                self._logged_publish_error = True
```

**Articulation acquisition** (parity_publishers.py:316-325 — copy as-is; the same `/World/UR5e/aic_unified_robot/root_joint` handle is the one Phase 2 writes to):

```python
try:
    from isaacsim.core.prims import Articulation
    art_root = f"{self._robot_xform_path}/root_joint"
    self._articulation = Articulation(prim_paths_expr=art_root)
    self._articulation.initialize()
    for idx, name in enumerate(self._articulation.dof_names or []):
        self._dof_index_by_name[name] = idx
except Exception as exc:
    print(f"[AIC-DT][parity] Articulation init failed: {exc!r}")
    self._articulation = None
```

**Inversions / Phase-2-only additions:**

1. **Subscribers** (NEW — Phase 1 has none): in `start()` after publisher creation, before physx subscription:
   ```python
   from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState
   from ros_gz_interfaces.msg import Contacts
   # QoS per RESEARCH.md "Don't Hand-Roll" table — match aic_controller.cpp:182
   cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=10)
   self._joint_cmd_sub = self._node.create_subscription(
       JointMotionUpdate, "/aic_controller/joint_commands",
       self._on_joint_cmd, cmd_qos)
   self._pose_cmd_sub = self._node.create_subscription(
       MotionUpdate, "/aic_controller/pose_commands",
       self._on_pose_cmd, cmd_qos)
   self._ctrl_state_pub = self._node.create_publisher(
       ControllerState, "/aic_controller/controller_state", cmd_qos)
   self._contacts_pub = self._node.create_publisher(
       Contacts, "/aic/gazebo/contacts/off_limit", cmd_qos)
   ```
2. **rclpy.spin_once(node, timeout_sec=0)** as step 1 of `_on_physics_step` (Phase 1 doesn't spin — it's pure publisher). Per D-08:
   ```python
   def _on_physics_step(self, dt: float):
       import rclpy
       rclpy.spin_once(self._node, timeout_sec=0)   # NEW
       if self._latest_joint_cmd is not None:
           self._apply_joint_cmd(self._latest_joint_cmd)
           self._latest_joint_cmd = None
       if self._latest_pose_cmd is not None:
           self._apply_pose_cmd(self._latest_pose_cmd)
           self._latest_pose_cmd = None
       self._publish_controller_state(...)
       self._publish_offlimit_contacts(...)
   ```
3. **ArticulationKinematicsSolver init** in `start()` (Phase 1 has no IK — it just reads articulation state). Code in RESEARCH.md "Pattern 5" (lines 350-396).
4. **omni.physx contact-report subscription** in `start()` (a SECOND `omni.physx` subscription on top of the physics-step one — these are independent event streams). Pattern in RESEARCH.md "Pattern 6" (lines 402-461).
5. **Subscriber callbacks** `_on_joint_cmd` / `_on_pose_cmd` — **must NOT apply commands directly**, only buffer to `self._latest_*_cmd` for the physics-tick to consume (avoids main-thread vs physics-thread races; mirrors Phase 1's read-on-physics-tick discipline). Code in RESEARCH.md lines 588-611, 695-700.
6. **D-11 silent drop discipline** (mirrored from parity_publishers.py:393-397's "log once" pattern): bad commands log debug + return, never raise.
7. **`stop()` extension**: tear down both `self._physx_sub` AND `self._contact_sub`; call `_node.destroy_subscription` for both subscribers (parity_publishers.py:344-368 only destroys publishers; the inversion adds subscriber teardown).

**Notable tricky bits to preserve from the analog:**

- `from omni.physx import get_physx_interface` (line 332) — **import this way, not `import omni.physx`**, to avoid creating a function-local `omni` binding that shadows module-level `import omni.usd` (parity_publishers.py:330-332 explicit comment).
- `_logged_publish_error` boolean pattern (line 395) — log-once on physics-step exceptions to avoid log spam.
- The articulation's joint values come back as torch.Tensor or numpy depending on backend; the `pos[0] if hasattr(pos[0], "__len__") else pos` defensive unwrap (line 426) — needed for write paths too.

---

### `exts/aic-dt/aic_dt/extension.py` (MCP atom integration, MODIFY)

**Analog:** Phase 1's PARITY-03/04 additions (the `setup_tf_publisher` + `setup_joint_state_publisher` 4-surface contract). The pattern is enumerated in extension.py:2768-2782 ("4-surface contract per atom (DX-02)").

**Surface 1 — `MCP_TOOL_REGISTRY` entry** (mirror lines 197-204):

```python
# extension.py:197-204 (analog)
"setup_tf_publisher": {
    "description": "Create the ROS2 TF publisher action graph for /tf (dynamic) and /tf_static (TRANSIENT_LOCAL via staticPublisher=True). Articulation root walks via ROS2PublishTransformTree.",
    "parameters": {}
},
"setup_joint_state_publisher": {
    "description": "Create the ROS2 /joint_states publisher action graph (sensor_msgs/JointState). Reads articulation state and emits ROS2 messages from the sim thread.",
    "parameters": {}
},
```

Phase 2 adds 2 entries before `setup_wrist_cameras` (~line 205). The `setup_offlimit_contacts` atom takes a `prim_paths` array parameter per RESEARCH.md "MCP atom signature" section (lines 549-560).

**Surface 2 — `MCP_HANDLERS` entry** (mirror lines 358-359):

```python
# extension.py:358-359 (analog)
"setup_tf_publisher": "_cmd_setup_tf_publisher",
"setup_joint_state_publisher": "_cmd_setup_joint_state_publisher",
```

Phase 2 adds 2 entries near the same neighborhood:

```python
"setup_controller_subscribers": "_cmd_setup_controller_subscribers",
"setup_offlimit_contacts": "_cmd_setup_offlimit_contacts",
```

**Surface 3 — `_cmd_<name>` method** (mirror lines 2730-2744):

```python
# extension.py:2730-2744 (analog)
def _cmd_setup_tf_publisher(self) -> Dict[str, Any]:
    try:
        self.setup_tf_publish_action_graph()
        return {"status": "success", "message": "TF publisher action graph created (/tf + /tf_static)"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def _cmd_setup_joint_state_publisher(self) -> Dict[str, Any]:
    try:
        self.setup_joint_state_publish_action_graph()
        return {"status": "success", "message": "JointState publisher action graph created on /joint_states"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}
```

Phase 2 mirrors with the new helper-delegate pattern (analog: extension.py:1538-1559's `_start_aic_parity_publishers`):

```python
def _cmd_setup_controller_subscribers(self) -> Dict[str, Any]:
    try:
        self._start_aic_controller_loop()  # delegates to shared manager (parity with Phase 1)
        return {"status": "success",
                "message": "AIC controller loop started (/aic_controller/joint_commands + /pose_commands subs, /controller_state pub)"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def _cmd_setup_offlimit_contacts(self, prim_paths: list = None) -> Dict[str, Any]:
    try:
        self._start_aic_controller_loop(off_limit_prims=prim_paths)
        return {"status": "success",
                "message": f"Off-limit contact subscription active ({len(prim_paths or [])} prims)"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}
```

**Helper method `_start_aic_controller_loop`** (mirror extension.py:1538-1559 verbatim, swap class name and module path):

```python
# extension.py:1538-1559 (analog)
def _start_aic_parity_publishers(self):
    """Start the shared AicParityPublishers manager (idempotent)."""
    try:
        from .parity_publishers import AicParityPublishers
    except Exception as exc:
        print(f"[AIC-DT][parity] Failed to import parity_publishers: {exc!r}")
        return
    if self._aic_parity_publishers is None:
        self._aic_parity_publishers = AicParityPublishers(
            robot_xform_path=self._articulation_root_prim_path
        )
    ok = self._aic_parity_publishers.start()
    if not ok:
        print("[AIC-DT][parity] Publisher start() returned False -- check rclpy availability.")
```

Phase 2 adds:

```python
def _start_aic_controller_loop(self, off_limit_prims: list = None):
    try:
        from .controller_loop import AicControllerLoop
    except Exception as exc:
        print(f"[AIC-DT][controller] Failed to import controller_loop: {exc!r}")
        return
    if self._aic_controller_loop is None:
        self._aic_controller_loop = AicControllerLoop(
            robot_xform_path=self._articulation_root_prim_path,
            off_limit_prims=off_limit_prims,
        )
    elif off_limit_prims is not None:
        # Allow per-call override of off_limit set
        self._aic_controller_loop.set_off_limit_prims(off_limit_prims)
    ok = self._aic_controller_loop.start()
    if not ok:
        print("[AIC-DT][controller] start() returned False -- check rclpy / aic_control_interfaces availability.")
```

And add the `__init__` field next to `self._aic_parity_publishers = None` at line 442:

```python
self._aic_controller_loop = None  # NEW — Phase 2
```

**Surface 4 — UI button** (mirror lines 549-551, in the "UR5e Robot Control" CollapsableFrame):

```python
# extension.py:549-551 (analog)
ui.Button("Setup Force Publisher", width=200, height=35, clicked_fn=self.setup_force_publish_action_graph)
ui.Button("Setup TF Publisher", width=200, height=35, clicked_fn=self.setup_tf_publish_action_graph)
ui.Button("Setup JointState Publisher", width=200, height=35, clicked_fn=self.setup_joint_state_publish_action_graph)
```

Add 2 lines below:

```python
ui.Button("Setup Controller Subscribers", width=220, height=35,
          clicked_fn=lambda: self._cmd_setup_controller_subscribers())
ui.Button("Setup Off-Limit Contacts", width=220, height=35,
          clicked_fn=lambda: self._cmd_setup_offlimit_contacts())
```

**`on_shutdown` extension** (mirror extension.py:3186-3192 verbatim, append a second block):

```python
# extension.py:3186-3192 (analog)
if self._aic_parity_publishers is not None:
    try:
        self._aic_parity_publishers.stop()
    except Exception as exc:
        print(f"[AIC-DT][parity] stop() failed in shutdown: {exc!r}")
    self._aic_parity_publishers = None
```

Phase 2 appends:

```python
if self._aic_controller_loop is not None:
    try:
        self._aic_controller_loop.stop()
    except Exception as exc:
        print(f"[AIC-DT][controller] stop() failed in shutdown: {exc!r}")
    self._aic_controller_loop = None
```

**`quick_start` chain insertion** (extension.py:1016-1024 — current ordering, Phase 2 adds new block AFTER step 3b "JointState Publisher" and BEFORE step 4 "UR5e Action Graph"):

```python
# extension.py:1016-1024 (analog — Phase 1 step 3a/3b)
# 3a. Setup TF publisher (Plan 06 / Phase 1 D-10) — /tf + /tf_static
print("--- Setting up TF Publisher (/tf + /tf_static) ---")
self.setup_tf_publish_action_graph()
await app.next_update_async()

# 3b. Setup JointState publisher (Plan 06 / Phase 1 D-11) — /joint_states
print("--- Setting up JointState Publisher (/joint_states) ---")
self.setup_joint_state_publish_action_graph()
await app.next_update_async()
```

Phase 2 inserts new step 3c/3d (the comment at line 988 already calls out this slot: "Phase 2 controller-loop: subscribes between JointState publisher and Force publisher"):

```python
# 3c. Phase 2 — controller subscribers + state publisher (PARITY-09/10/11)
print("--- Setting up AIC Controller Subscribers ---")
self._start_aic_controller_loop()
await app.next_update_async()

# 3d. Phase 2 — off-limit contact event publisher (PARITY-06)
# Re-uses the same manager; off_limit_prims=None → uses controller_loop.py's default set
print("--- Setting up Off-Limit Contact Subscription ---")
# (already started above; this is a no-op re-entry but keeps the chain explicit)
```

(Both atoms share the same `AicControllerLoop` instance per RESEARCH.md "Single rclpy node owning all 4 topics" pattern; the second call is idempotent.)

---

### `exts/aic-dt/scripts/smoke_test_aic_controller.py` (NEW, external Python 3.10 verifier)

**Analog:** `exts/aic-dt/scripts/smoke_test_aic_parity.py` (188 LOC, all 13/13 checks pass on Phase 1 close).

**Module docstring + imports pattern** (smoke_test_aic_parity.py:1-43 — copy exactly, expand imports):

```python
# smoke_test_aic_parity.py:1-43 (analog)
"""Phase 1 topic-surface smoke test — runs OUTSIDE Isaac Sim's Python.

Uses /opt/ros/humble Python 3.10's rclpy + tf2_ros (the same Python that
aic_adapter and CheatCode use). If this test passes, Phase 1's published
ROS surface can be consumed by the actual AIC stack with no special path
manipulation.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros.transform_listener import TransformListener
```

Phase 2 expands imports to include:

```python
from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, ControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Contacts
```

> **CRITICAL PRECONDITION:** smoke_test_aic_parity.py runs against `/opt/ros/humble` Python 3.10 successfully because all its messages are stock. Phase 2's smoke test ALSO needs `aic_control_interfaces` AND `ros_gz_interfaces` available for Python 3.10 (`/opt/ros/humble`). The Phase 1 workspace targets Python 3.11 — Phase 2 needs a SECOND build target for 3.10, OR the smoke test needs to source the AIC pixi env. Recommendation in RESEARCH.md is to add `aic_control_interfaces` to BOTH the 3.11 workspace (for Isaac Sim) AND verify it's importable from `/opt/ros/humble` (for the smoke test) — the AIC pixi env's Python 3.12 build is unsuitable for both ABIs.

**Subscriber pattern + spin** (smoke_test_aic_parity.py:59-87 — `SmokeTester(Node)` class):

```python
# smoke_test_aic_parity.py:59-71 (analog)
class SmokeTester(Node):
    def __init__(self):
        super().__init__("aic_parity_smoke_tester")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._js_msgs = []
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def _on_joint_state(self, msg):
        self._js_msgs.append(msg)
```

Phase 2 mirrors with publishers + new subscribers:

```python
class ControllerSmokeTester(Node):
    def __init__(self):
        super().__init__("aic_controller_smoke_tester")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._js_msgs = []
        self._ctrl_state_msgs = []
        self._contact_msgs = []
        # Subscribers
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(ControllerState, "/aic_controller/controller_state", self._on_cs, 10)
        self.create_subscription(Contacts, "/aic/gazebo/contacts/off_limit", self._on_ct, 10)
        # Publishers (the Phase 2 inversion)
        self._joint_cmd_pub = self.create_publisher(JointMotionUpdate, "/aic_controller/joint_commands", 10)
        self._pose_cmd_pub = self.create_publisher(MotionUpdate, "/aic_controller/pose_commands", 10)
```

**Step pattern + report() helper + main verdict** (smoke_test_aic_parity.py:73-184 — copy verbatim, swap content per D-12 7-step contract):

```python
# smoke_test_aic_parity.py:73-77 (analog)
def report(label, passed, detail=""):
    icon = "✓" if passed else "✗"
    print(f"  {icon} {label}{(': ' + detail) if detail else ''}")
    return passed

# smoke_test_aic_parity.py:178-184 (analog) — verdict
passed = sum(1 for r in results if r)
total = len(results)
print(f"\n{passed}/{total} checks passed.")
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if passed == total else 1)
```

Phase 2's 7 steps per D-12:
1. Connect to Isaac Sim (already running aic-dt instance assumed; smoke test does NOT bring up Isaac Sim — same as Phase 1).
2. Capture initial `/joint_states` snapshot (use Phase 1 publisher).
3. Publish a `JointMotionUpdate` with a 0.05-rad delta on `shoulder_lift_joint`; spin 2s; verify new `/joint_states` shows the moved position (within 0.02 rad).
4. Publish a `MotionUpdate` with a 0.05m delta in `base_link` frame; spin 2s; verify `lookup_transform("base_link", "tool0")` shows updated translation.
5. Subscribe `/aic_controller/controller_state`; verify nonzero `tcp_pose` AND nonzero `tcp_error` after step 4.
6. Move robot into a known off-limit prim via `JointMotionUpdate` (or direct `execute_python_code` via MCP); spin 2s; verify `/aic/gazebo/contacts/off_limit` received nonempty Contacts.
7. Verdict (mirror smoke_test_aic_parity.py:178-184 exactly).

**Reusable helpers from Phase 1's smoke test:**
- `vec_norm(t)` (line 56) — euclidean magnitude for translation diff plausibility checks.
- The "warm up TF buffer for Ns" idiom (lines 84-87) — needed before any `lookup_transform` calls.

---

### `exts/aic-dt/docs/aic-msgs-setup.md` (NEW, docs)

**Analog:** `exts/aic-dt/docs/rclpy-setup.md` (80 LOC). Same audience (the developer setting up a fresh repo), same shape (Why → Prerequisites → Build → Configuration → Usage → Verification).

**Why section pattern** (rclpy-setup.md:5-13):

```markdown
# rclpy Setup for Isaac Sim 5.0

Isaac Sim 5.0 uses Python 3.11, but ROS2 Humble ships rclpy compiled for Python 3.10. This guide builds a Python 3.11-compatible ROS2 Humble workspace so that `import rclpy` works inside Isaac Sim extensions.

## Why

The Isaac Sim ROS2 bridge (`isaacsim.ros2.bridge`) uses C++ internally and works without rclpy. However, rclpy enables:

- **Subscriber-aware cameras** — detect when a ROS2 node subscribes to a camera topic and only render when needed
- **ROS2 services** — request/response pattern from within the extension
- **Direct Python callbacks** — handle incoming ROS2 messages without OmniGraph polling
- **Custom message types** — use any ROS2 message type directly
```

Phase 2 mirrors with the D-05 landmine context:

```markdown
# AIC Custom Messages Setup for Isaac Sim 5.0

Phase 2's controller loop subscribes to `aic_control_interfaces/{JointMotionUpdate, MotionUpdate, ControllerState}` and publishes `ros_gz_interfaces/Contacts`. Neither package ships in the IsaacSim-ros_workspaces base build; both must be added to the Python 3.11 workspace.

## Why not use the AIC pixi env directly?

The `~/Documents/aic` repo's pixi env builds `aic_control_interfaces` against Python 3.12. Isaac Sim 5.0 runs Python 3.11. CPython C-extension `.so` files (which ROS message typesupport ships as) are NOT ABI-compatible across minor versions. Loading the pixi env's modules from Isaac Sim's Python 3.11 fails at the typesupport-`.so` import.

**Solution:** vendor the source into `~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/` and rebuild against Python 3.11.
```

**Build command pattern** (rclpy-setup.md:25-37):

```bash
# rclpy-setup.md:25-37 (analog)
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git ~/IsaacSim-ros_workspaces
cd ~/IsaacSim-ros_workspaces
git checkout IsaacSim-5.0.0-full
bash build_ros.sh -d humble -v 22.04
```

Phase 2 mirrors with the new packages step (full sequence in RESEARCH.md lines 134-149):

```bash
cd /tmp && git clone -b humble https://github.com/gazebosim/ros_gz.git
cp -r /tmp/ros_gz/ros_gz_interfaces ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/
cp -r ~/Documents/aic/aic_interfaces/aic_control_interfaces ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src/
cd ~/IsaacSim-ros_workspaces && bash build_ros.sh -d humble -v 22.04
ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/aic_control_interfaces/*.so
ls ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/ros_gz_interfaces/*.so
```

**Verification section pattern** (rclpy-setup.md:70-80):

```markdown
## Verification

From the Isaac Sim Script Editor (Window > Script Editor), run:

```python
import rclpy
from sensor_msgs.msg import Image
print("rclpy OK")
```
```

Phase 2 mirrors:

```python
import rclpy
from aic_control_interfaces.msg import JointMotionUpdate
from ros_gz_interfaces.msg import Contacts
print("AIC custom messages OK")
```

**LD_LIBRARY_PATH addition** (D-13 — extends rclpy-setup.md:50 with the new path):

```bash
# rclpy-setup.md:50 (analog)
export LD_LIBRARY_PATH="$HOME/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/lib:$LD_LIBRARY_PATH"
```

Phase 2 documents adding the AIC pixi-env path for ros-kilted shared libs (per D-13):

```bash
# Phase 2 D-13 — for AIC's ros-kilted runtime libs (NOT the Python pkg, just .so deps)
export LD_LIBRARY_PATH="$HOME/Documents/aic/.pixi/envs/default/lib:$LD_LIBRARY_PATH"
```

---

### `exts/aic-dt/scripts/snapshot_aic_eval_offlimit.sh` (NEW, helper for D-10 discovery)

**Analog:** `exts/aic-dt/scripts/snapshot_aic_eval.sh` (the existing topic-surface snapshot script, 40+ LOC).

**Set-euo + DEST pattern** (snapshot_aic_eval.sh:1-22):

```bash
#!/usr/bin/env bash
# scripts/snapshot_aic_eval.sh — capture live Gazebo aic_eval topic surface (D-01, D-14)
#
# Usage: ./snapshot_aic_eval.sh [DEST]
#   DEST defaults to .planning/phases/01-foundation-parity/snapshot
set -euo pipefail

DEST="${1:-.planning/phases/01-foundation-parity/snapshot}"
mkdir -p "$DEST"
echo "Snapshot DEST: $DEST"

# 1. Idempotent cleanup of any prior container
docker rm -f aic_eval 2>/dev/null || true

# 2. Bring up aic_eval headless
echo "Starting aic_eval container (headless, ground_truth=true)..."
docker run -d --name aic_eval --gpus all --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host --privileged \
  ghcr.io/intrinsic-dev/aic/aic_eval:latest \
  ground_truth:=true start_aic_engine:=true gazebo_gui:=false

# 3. Wait for Gazebo + controller_manager
sleep 35
```

Phase 2 mirrors the bringup but ALSO launches a CheatCode trial (per RESEARCH.md "Pre-flight" section lines 531-540) and captures the off_limit topic instead of the broad `topic list`:

```bash
# Pull aic_model + start CheatCode (the trial that exercises the off_limit topic)
cd "$HOME/Documents/aic" && ./scripts/run_cheatcode.sh headless &
sleep 60   # wait for stack + first contact-relevant motion

# Capture off_limit contacts for 120s (covers a full cheatcode cycle)
docker exec aic_eval bash -c '
  source /opt/ros/kilted/setup.bash &&
  timeout 120 ros2 topic echo /aic/gazebo/contacts/off_limit
' | tee "$DEST/aic_eval_offlimit_capture.txt"

# Tear down
cd "$HOME/Documents/aic" && ./scripts/run_cheatcode.sh stop
```

The destination should be `.planning/phases/02-controller-loop/snapshot/`.

---

### `exts/aic-dt/scripts/build_aic_msgs.sh` (NEW, OPTIONAL helper for D-05 fix)

**Analog:** None bash-scripted in repo — `rclpy-setup.md` has the equivalent commands as documentation. Recommendation: **skip this file** unless the planner judges that scripted automation reduces error vs. documented commands. The build runs Docker → ~10-20 minutes → low frequency (once per dev setup); doc-only is likely sufficient. If created:

**Pattern source:** `rclpy-setup.md:25-37` (the Phase 1 build command sequence) wrapped in a shell script with `set -euo pipefail`, the same git-clone + cp + build sequence as `aic-msgs-setup.md` will document.

```bash
#!/usr/bin/env bash
# scripts/build_aic_msgs.sh — Phase 2 D-05 fix: rebuild Python 3.11 workspace
#                              with aic_control_interfaces + ros_gz_interfaces.
# See exts/aic-dt/docs/aic-msgs-setup.md for full background.
set -euo pipefail
WS="$HOME/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/src"
[[ -d "$WS" ]] || { echo "ERROR: workspace not found at $WS — run rclpy-setup.md first"; exit 1; }

# 1. Clone & vendor ros_gz_interfaces
if [[ ! -d "$WS/ros_gz_interfaces" ]]; then
  cd /tmp && rm -rf ros_gz && git clone -b humble https://github.com/gazebosim/ros_gz.git
  cp -r /tmp/ros_gz/ros_gz_interfaces "$WS/"
fi

# 2. Vendor aic_control_interfaces
[[ -d "$WS/aic_control_interfaces" ]] || \
  cp -r "$HOME/Documents/aic/aic_interfaces/aic_control_interfaces" "$WS/"

# 3. Rebuild
cd "$HOME/IsaacSim-ros_workspaces" && bash build_ros.sh -d humble -v 22.04

# 4. Sanity-check 3.11 ABI
INSTALL="$HOME/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages"
ls "$INSTALL/aic_control_interfaces/" "$INSTALL/ros_gz_interfaces/" || \
  { echo "ERROR: install dirs missing — workspace build failed"; exit 1; }
echo "OK: aic_control_interfaces + ros_gz_interfaces built for Python 3.11"
```

---

## Shared Patterns

### Path discipline for custom ROS messages (the most-copied pattern)

**Source:** `parity_publishers.py:32-124`
**Apply to:** `controller_loop.py` only (extension.py and smoke_test don't need it; they depend on it transitively via the controller_loop import).

The 3-piece pattern:
1. **Module-level removal of stale 3.10 paths + prepend of 3.11 paths** (lines 32-59) — runs at import time.
2. **`_force_python311_ros_paths()` helper** (lines 62-86) — re-runs at `start()` time because Carbonite re-injects 3.10 paths post-startup.
3. **`_ensure_rclpy_clean_import()` helper** (lines 89-124) — evicts half-loaded ROS modules from `sys.modules` so the next import uses the 3.11 ABI.

Phase 2 extension: add `aic_control_interfaces` and `ros_gz_interfaces` to the `_ROS_PREFIXES` tuple at line 112 (the eviction set).

### Idempotent class lifecycle (`start()` calls `stop()` first)

**Source:** `parity_publishers.py:248` and `parity_publishers.py:344-368`
**Apply to:** `controller_loop.py` (the AicControllerLoop class)

```python
def start(self):
    self.stop()  # idempotent reset — safe to call repeatedly on hot-reload
    # ... initialize everything ...

def stop(self):
    # Tear down physx_sub, contact_sub, node, all field handles → None
    ...
```

### "Log once, never raise from physics callback"

**Source:** `parity_publishers.py:393-397`
**Apply to:** `controller_loop.py._on_physics_step`, `controller_loop.py._on_contact_event` (omni.physx callback), `controller_loop.py._on_joint_cmd`, `controller_loop.py._on_pose_cmd`

```python
# parity_publishers.py:393-397 (analog)
except Exception as exc:
    if not getattr(self, "_logged_publish_error", False):
        print(f"[AIC-DT][parity] publish error: {exc!r}")
        self._logged_publish_error = True
```

Mirror per-callback (use distinct flag names: `_logged_apply_error`, `_logged_contact_error`, etc.).

### MCP atom 4-surface contract

**Source:** `extension.py:2768-2782` (DX-02 contract enumerated as a comment block) and the 7 `spawn_*` atoms at extension.py:215-280 (registry), 363-369 (handlers), 2785+ (methods), 605-641 (UI buttons).

**Apply to:** Both `setup_controller_subscribers` AND `setup_offlimit_contacts`. The audit script `scripts/audit_dx02.py` enforces this — Phase 2 atom additions need to pass `audit_dx02.py` (Phase 1 closed at 27 PRESENT × 4 surfaces; Phase 2 brings it to 29 × 4).

### Manager pattern: helper method delegated from atom + cleaned in on_shutdown

**Source:** `extension.py:1538-1559` (`_start_aic_parity_publishers`) + `extension.py:3186-3192` (on_shutdown teardown)
**Apply to:** `extension.py` — add `_start_aic_controller_loop` helper + symmetric on_shutdown block + `self._aic_controller_loop = None` field init at line 442.

This pattern means MULTIPLE atoms can share ONE manager instance (Phase 1 has 2 atoms `setup_tf_publisher` + `setup_joint_state_publisher` both delegating to `_start_aic_parity_publishers`; Phase 2 mirrors with 2 atoms `setup_controller_subscribers` + `setup_offlimit_contacts` both delegating to `_start_aic_controller_loop`).

### `quick_start` chain insertion

**Source:** `extension.py:982-1100` — the documented order is "Phase 2 controller-loop slot is between JointState publisher and Force publisher" (extension.py:988 explicit comment).
**Apply to:** Phase 2 inserts new step 3c between current 3b and 4. The `await app.next_update_async()` between every step is non-negotiable — see line 1007's "Bisection on 2026-05-01" comment block (omitting the awaits wedges Kit's main thread).

### "Live aic_eval container is the truth, NOT the YAML config"

**Source:** `CLAUDE.md` (the discovery note dated 2026-05-01) + `snapshot_aic_eval.sh`
**Apply to:** D-10 off-limit prim discovery (per RESEARCH.md "Pre-flight: which prims are off-limit?" section). The snapshot_aic_eval_offlimit.sh script captures live truth; the `docs/offlimit-prim-mapping.md` derived from it (NEW doc, not classified above — small enough to author inline during plan execution) maps Gazebo entity names to Isaac Sim USD paths.

---

## No Analog Found

None — every Phase 2 file maps to a Phase 1 analog. The phase is structurally an inversion of Phase 1 with two algorithmic additions (Lula IK and omni.physx contact-report) that are themselves drop-ins from canonical sources (RESEARCH.md "Pattern 5" + the `robot-collision-forensics` skill's 185-LOC `contact_subscription.py`).

| Algorithmic addition | Drop-in source | Notes |
|----------------------|----------------|-------|
| Lula IK setup | RESEARCH.md "Pattern 5" lines 350-396 (verbatim Isaac Sim 5.0 idiom) + bundled UR5e templates at `~/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/.../motion_policy_configs/universal_robots/ur5e/` | Use `LulaKinematicsSolver` (NOT RmpFlow) for first cut per RESEARCH "Alternatives Considered" |
| omni.physx contact-report | `~/.claude/skills/robot-collision-forensics/scripts/contact_subscription.py` (185 LOC) | Import or copy whole; treat as canonical per CLAUDE.md "robot-collision-forensics skill is authoritative for D-03" |

---

## Metadata

**Analog search scope:**
- `exts/aic-dt/aic_dt/` (parity_publishers.py, extension.py)
- `exts/aic-dt/scripts/` (smoke_test_aic_parity.py, snapshot_aic_eval.sh, audit_dx02.py)
- `exts/aic-dt/docs/` (rclpy-setup.md, README.md, topic-parity-reference.md)
- `.planning/phases/01-foundation-parity/` (01-SUMMARY.md, 01-CONTEXT.md, 01-VERIFICATION.md)

**Files scanned:** 9 source files + 4 docs + 4 planning files = 17

**Pattern extraction date:** 2026-05-03

**Cross-reference for the planner:**
- The "no source edits" rule: PATTERNS.md is the only file authored here. All analog references are line-cited to existing files; the planner is responsible for the actual `controller_loop.py` authoring + extension.py edits in Phase 2 plans.
- The 6-plan recommendation in RESEARCH.md (lines 25) maps cleanly onto the file inventory above:
  - Plan 1: workspace rebuild (D-05 fix) → touches `aic-msgs-setup.md` + `build_aic_msgs.sh` (optional)
  - Plan 2: skeleton + 2 atoms (D-01/D-04) → touches `controller_loop.py` (skeleton) + `extension.py` (4-surface × 2)
  - Plan 3: joint command application (PARITY-09) → touches `controller_loop.py` only
  - Plan 4: pose command + Lula IK (PARITY-10) → touches `controller_loop.py` only
  - Plan 5: ControllerState publish (PARITY-11) → touches `controller_loop.py` only
  - Plan 6: omni.physx contacts + Contacts publish (PARITY-06) + smoke test (D-12) → touches `controller_loop.py` + NEW `smoke_test_aic_controller.py` + NEW `snapshot_aic_eval_offlimit.sh`

## PATTERN MAPPING COMPLETE
