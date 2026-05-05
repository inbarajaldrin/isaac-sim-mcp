# Plan 03-04 + 03-05 contract — extracted from `aic_scoring/ScoringTier2`

**Captured:** 2026-05-05
**Source:** `~/Documents/aic/aic_scoring/{include,src}/aic_scoring/ScoringTier2.{hh,cc}`

The /scoring/* topics referenced in `sample_config.yaml` ARE real subscribers
in `aic_scoring/ScoringTier2.cc` — just not loaded in standard `aic_eval`
ground_truth mode (Plan 03-01 captured against). Tier2 scoring activates
them; Phase 4 trial loader will exercise this path end-to-end.

## /scoring/tf publisher contract (PARITY-08)

- **Topic:** `/scoring/tf`
- **Type:** `tf2_msgs/msg/TFMessage`
- **Direction:** Isaac Sim → ScoringTier2 (subscriber)
- **Content:** TFMessage of CABLE-RELATED transforms specifically.
  `ScoringTier2.cc::TfCallback` distinguishes:
  - `/tf` → gripper transforms (cableTfReceived stays false)
  - `/scoring/tf` → cable transforms (cableTfReceived = true)
- **Frame parent:** `world` (per the static `world → aic_world` reference also published by Phase 1)
- **Frames to publish:**
  - Cable plug-end frame (link_20 pose; child_frame_id = TBD literal — likely `cable/sc_plug_link` or similar)
  - Port frames (children of task_board; child_frame_id MUST contain "task_board" substring per `ScoringTier2.cc:51-53` — that's the trigger for `setTransform(tf, "scoring", true)` static registration)
  - Per-cable-link frames (optional but matches /objects_poses_real expectation)

## /objects_poses_real publisher contract (re-scoped PARITY-08 alt)

- **Topic:** `/objects_poses_real`
- **Type:** `tf2_msgs/msg/TFMessage`
- **Direction:** Isaac Sim → grasp_points_publisher (subscriber, count=1 per Plan 03-01 capture)
- **Content:** TFMessage of all object + cable + task_board poses (broader than /scoring/tf)
- **Frame parent:** `world`

**Recommendation:** publish identical content on BOTH /scoring/tf + /objects_poses_real. Both are TFMessage; both want cable + task_board frames; the marginal cost is one extra `pub.publish(msg)` per tick.

## /scoring/insertion_event publisher contract (PARITY-07)

- **Topic:** `/scoring/insertion_event`
- **Type:** `std_msgs/msg/String`
- **Direction:** Isaac Sim → ScoringTier2 (subscriber)
- **Payload:** `data = port_namespace_string` — identifies WHICH port was inserted into.
  - From `ScoringTier2.cc::InsertionEventCallback`:
    ```cpp
    void ScoringTier2::InsertionEventCallback(const StringMsg &_msg) {
      // \todo(iche033) For now, assume only one insertion event per task
      // Mark insertion completion as true as soon as one insertion is done.
      this->insertionPortNamespace = _msg.data;
    }
    ```
  - So payload format: bare port-name string, no JSON/yaml.
- **Trigger condition:** publish ONCE when contact between plug-end link (link_20) and a port link is detected for ≥5 ticks (~80ms).
  - Use omni.physx contact-report subscription pattern from Plan 02-06's PARITY-06 in `controller_loop._setup_contact_subscription`.
  - Filter: only fire on (plug_end_link, port_link) pair (NOT generic off-limit).
  - Re-arm only when plug separates (gap ≥ N ticks without contact).
- **Per-port detection:** when contact fires, identify WHICH port the plug touched (e.g. by reading the actor1 path, extract port name like "sc_port_1" or "nic_card_port_a"). Set the String.data to that port name.

## Implementation hints for parity_publishers.py

Add to `start()` (under existing publisher creation):
```python
self._scoring_tf_pub = self._node.create_publisher(
    TFMessage, "/scoring/tf", scoring_qos)
self._objects_poses_pub = self._node.create_publisher(
    TFMessage, "/objects_poses_real", scoring_qos)
self._scoring_event_pub = self._node.create_publisher(
    String, "/scoring/insertion_event", scoring_qos)
```

Per-tick from `_on_physics_step`:
```python
self._publish_scoring_tf(now)  # cable + task_board frames
self._publish_objects_poses(now)  # same content as scoring_tf, just published twice
self._maybe_publish_insertion_event()  # checks contact-event deque
```

Add:
- `_setup_insertion_contact_subscription()` — mirrors `controller_loop._setup_contact_subscription` from Plan 02-06; filter for plug↔port pairs; appends to `INSERTION_CONTACT_EVENTS` deque.
- `_on_insertion_contact_event(headers, data)` — physics-thread O(1) append.
- `_maybe_publish_insertion_event()` — sustained-contact gate + edge-detection; sets self._insertion_event_armed flag to suppress repeats.

## Frame name discovery (TODO)

The literal `child_frame_id` strings the scoring system expects are NOT documented
in ScoringTier2 source (it just substring-matches "task_board"). To get the exact
names, run `aic_eval` in tier2 mode (NOT the default ground_truth mode I tested),
capture `/scoring/tf` echo with publishers added (or grep `aic_gazebo` source for
the Gazebo-side publisher's frame conventions).

For now (defensive defaults that match aic_engine config conventions):
- `world → cable/sc_plug_link` (cable plug end, dynamic)
- `world → cable/sc_port_link` (port the plug should insert into, dynamic)
- `world → task_board_base` (task_board static — triggers ScoringTier2's "task_board" substring match)
- `world → task_board_sc_port_1`, `world → task_board_sc_port_2`, etc. (per-port frames, also "task_board" substring)
- `world → task_board_nic_card` (NIC card mount, also "task_board" substring)
