# Bug Report: ArUco marker TF frames missing from `view_frames`

**Date:** 2026-09-01
**Symptom reported by:** `ros2 run tf2_tools view_frames`
**Workspace:** `~/Desktop/robotics-ws`

---

## Symptom

Running

```bash
ros2 run tf2_tools view_frames
```

produced a TF tree (`frames_*.gv`) that contained **no `aruco_marker_*` frames**.
The static tree was otherwise healthy:

- `odom -> base_footprint -> base_link` (dynamic, odom broadcaster working)
- `base_link -> camera_link_1` / `camera_link_2` (cameras present as static TF)
- Both arm trees (`arm_base_link`, ...) present

Expected (per `nav_nodes/aruco_detector_node.py`) but missing:

```
camera_link_1 -> aruco_marker_<id>_front
camera_link_2 -> aruco_marker_<id>_rear
```

---

## Root causes

### Cause 1 — Invalid marker textures (primary)

The textures applied to the marker panels in `multiroom.sdf` were **not valid
ArUco markers** — they were hand-drawn concentric squares:

- `src/robot_description/models/aruco_marker_0/materials/textures/aruco_0.png`
- `src/robot_description/models/aruco_marker_1/materials/textures/aruco_1.png`

**Diagnosis method:**

```python
import cv2
g = cv2.cvtColor(cv2.imread(png), cv2.COLOR_BGR2GRAY)
corners, ids, _ = cv2.aruco.detectMarkers(g, cv2.aruco.getPredefinedDictionary(0))
# ids -> None for both files
```

Exhaustive test across **all predefined dictionaries (0-11), normal/inverted,
multiple scales, and added white borders (0-80 px)**: `NOTHING DETECTED`.
Run-length analysis of the pixel structure (42 px symmetric black/white
rings on a 377x377 image) confirmed concentric squares, not a 4x4-bit
dictionary marker with its 1-module black border.

Since `aruco_detector_node.py` broadcasts TF **only when `detectMarkers`
returns an ID**, no `aruco_marker_*` frames were ever published.

### Cause 2 — Segfault in `cv2.aruco.detectMarkers` when passing `DetectorParameters`

`aruco_detector_node.py` called:

```python
self.aruco_params = cv2.aruco.DetectorParameters()
corners, ids, _ = cv2.aruco.detectMarkers(
    gray, self.aruco_dict, parameters=self.aruco_params)
```

On this machine (OpenCV **4.6.0**, Python 3.12, numpy 1.26.4) this **hard-crashes
with SIGSEGV (exit 139)** on the first processed image — verified for both the
keyword (`parameters=p`) and positional (`None, None, p`) forms:

```
variant [cv2.aruco.detectMarkers(gray, d)]                  exit=0   -> ids [[0]]  OK
variant [cv2.aruco.detectMarkers(gray, d, parameters=p)]    exit=139 SEGFAULT
variant [cv2.aruco.detectMarkers(gray, d, None, None, p)]   exit=139 SEGFAULT
```

So even with valid markers in view, the detector node would have died the
moment the first camera frame arrived.

> Rule for this environment (OpenCV 4.6.0 legacy `cv2.aruco` API):
> **never pass a `DetectorParameters` object to `detectMarkers`** — omit it to
> get defaults. (`cv2.aruco.ArucoDetector`, used in newer OpenCV, is also
> unavailable in 4.6.0.)

### Not the problem (ruled out during diagnosis)

- ✅ Bridge topics match node subscriptions:
  `/camera/front|rear/image_raw` + `.../camera_info` (`gazebo.launch.py` ↔ node)
- ✅ QoS compatible (node uses BEST_EFFORT/VOLATILE matching the bridge)
- ✅ `camera_link_1` / `camera_link_2` frames exist in the TF tree and match
  the `<gz_frame_id>` set in `robot.urdf.xacro`
- ✅ `dict_id=0` (DICT_4X4_50) matches the intended dictionary; `cv2.aruco`
  module present
- ✅ `cv_bridge`, `tf2_ros.TransformBroadcaster` usage correct
- ✅ Node waits for `CameraInfo` intrinsics before pose estimation (`K is None` guard)
- ⚠️ Timing note: the original `frames_*.gv` was recorded at sim time ~5.8 s,
  while the detector starts via `TimerAction(period=6.0)` — so it was also
  captured before the node had even started. Marker frames are dynamic and
  only persist in the TF buffer (~10 s) while a marker is actually detected.

---

## Fixes applied

### 1. Regenerated valid DICT_4X4_50 marker textures

```python
import cv2
dict4x4 = cv2.aruco.getPredefinedDictionary(0)   # DICT_4X4_50
for mid in [0, 1]:
    marker = cv2.aruco.drawMarker(dict4x4, mid, 252, borderBits=1)  # 6 modules @ 42 px
    out = cv2.copyMakeBorder(marker, 42, 42, 42, 42,
                             cv2.BORDER_CONSTANT, value=255)        # 1-module white quiet zone
    cv2.imwrite(f'.../aruco_marker_{mid}/materials/textures/aruco_{mid}.png', out)
```

- IDs 0 and 1, 336x336 px, black border + white quiet zone
- Quiet zone included because the panels sit flush on a **brown shelf / wall**
  (medium contrast) — a border-less marker is unreliable there
- Verified: `detectMarkers` -> `ids = [0]` and `[1]`

### 2. Removed the segfaulting `parameters=` argument

`src/nav_nodes/nav_nodes/aruco_detector_node.py`:

```python
# NOTE: do NOT pass a DetectorParameters object here. This OpenCV
# 4.6.0 build segfaults when `parameters` is provided (verified:
# both keyword and positional forms crash). Omitting it uses the
# default detector parameters, which work correctly.
corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
```

`self.aruco_params = cv2.aruco.DetectorParameters()` removed from
`ArucoDetectorNode.__init__`.

Rebuilt: `colcon build --packages-select nav_nodes` (install copy verified updated).

### 3. Kept `marker_size` consistent with the texture geometry

The detectable marker is the **outer edge of the black border**, which with the
quiet zone spans 6/8 of the 0.15 m panel face:

```
detectable marker size = 0.15 * (6/8) = 0.1125 m
```

`src/robot_description/launch/arm_gazebo.launch.py`:

```python
parameters=[{
    'marker_size': 0.1125,   # must match texture geometry (was 0.15)
    'dict_id':     0,        # DICT_4X4_50
}],
```

If this is left at 0.15 while the texture has a quiet zone, all pose
estimates (`tvec`) come out ~25% too close — breaking docking distances.

`multiroom.sdf` comment block updated to document the 0.1125 m detectable size.

---

## Verification

| Check | Result |
|---|---|
| `detectMarkers` on new textures via the node's exact pipeline (grayscale, default params) | IDs `[0]`, `[1]` detected, no crash |
| `detectMarkers` with `parameters=DetectorParameters()` | SEGFAULT confirmed -> argument removed from node |
| `colcon build --packages-select nav_nodes` | OK, install tree updated |
| `python3 -m py_compile arm_gazebo.launch.py` | OK |
| `import nav_nodes.aruco_detector_node` from install tree | OK |

---

## How to confirm the fix at runtime

1. Launch the sim: `ros2 launch robot_description arm_gazebo.launch.py`
2. Drive the robot so a marker is in view of the front or rear camera.
3. Sanity-check detection: `ros2 run rqt_image_view rqt_image_view` on
   `/aruco/front/debug` (or `/aruco/rear/debug`) — marker outline + pose axes
   should be drawn.
4. While a marker is visible:

   ```bash
   ros2 run tf2_tools view_frames
   ```

   The graph should now include `camera_link_1 -> aruco_marker_0_front`
   (and/or `camera_link_2 -> aruco_marker_<id>_rear`).

**Reminder:** marker TF frames are transient — they appear only while the
marker is being detected and age out of the TF buffer after ~10 s. Run
`view_frames` during detection, not right after launch.

---

# Bug Report: `arm_controller_node` pick-and-place sequence never executes

**Date:** 2026-09-02
**Symptom reported by:** `ros2 run nav_nodes arm_controller_node`
**Workspace:** `~/Desktop/robotics-ws`

---

## Symptom

The node started normally and printed:

```
[INFO] [arm_controller_node]: arm_controller_node ready — waiting on /arm1/trigger
```

but after a trigger was received on `/arm1/trigger`, the pick-and-place
sequence never played correctly: steps fired repeatedly at overlapping
intervals, ran out of order, and `/arm1/done` was published prematurely and
multiple times — the arm received conflicting trajectories.

---

## Root cause — rclpy timers repeat unless destroyed (primary)

In `src/nav_nodes/nav_nodes/arm_controller_node.py`, the step chain was
scheduled with:

```python
def _schedule_next(self, delay_secs):
    self.create_timer(delay_secs, self._execute_step)

def _execute_step(self):
    # Cancel this one-shot timer (destroy after firing)
    if self._step_idx >= len(self._steps):
        ...
```

The comment claimed a *one-shot* timer, but **nothing was ever cancelled or
destroyed**. In rclpy, `Node.create_timer()` creates a **repeating** timer —
it keeps firing at its period forever unless `timer.cancel()` /
`node.destroy_timer()` is called.

Consequences once a trigger arrived:

- every `_schedule_next()` call stacked a **new** timer on top of the ones
  still running, so `N` timers were all firing `N` times per period,
- `_step_idx` advanced once per *each* live timer firing (N increments per
  tick), so steps executed out of order and far too fast,
- `done` was published prematurely and repeatedly,
- the arm got conflicting trajectories and the sequence never completed.

### Not the problem (ruled out during diagnosis)

- ✅ Node builds, runs, and spins — `ros2 run` reached the ready state
- ✅ Topic names match: `/arm1/arm_controller/joint_trajectory` and
  `/arm1/gripper_controller/joint_trajectory` match the controllers in
  `arm_controllers.yaml` under the `/arm1` namespace used in
  `arm_gazebo.launch.py`
- ✅ Joint names in `ARM_JOINTS` / `GRIPPER_JOINTS` match the controller yaml
  exactly
- ✅ Trigger/done topics (`/arm1/trigger`, `/arm1/done`) consistent

---

## Fix applied

`src/nav_nodes/nav_nodes/arm_controller_node.py` — store the timer handle and
make each step a true one-shot:

```python
def _schedule_next(self, delay_secs):
    # NOTE: rclpy timers repeat by default. The handle is stored so the
    # timer can be cancelled/destroyed in _execute_step (one-shot behaviour).
    self._timer = self.create_timer(delay_secs, self._execute_step)

def _execute_step(self):
    # One-shot: cancel and destroy the timer that just fired, otherwise it
    # keeps firing forever and stacks timers on every step.
    if self._timer is not None:
        self._timer.cancel()
        self.destroy_timer(self._timer)
        self._timer = None

    if self._step_idx >= len(self._steps):
        ...
```

`self._timer = None` is also initialised in `_run_sequence` before the chain
starts.

Rebuilt: `colcon build --packages-select nav_nodes` (build and install trees
verified in sync with the source).

---

## Verification

| Check | Result |
|---|---|
| `ros2 run nav_nodes arm_controller_node` | Node reaches ready state |
| `ros2 topic pub --once /arm1/trigger std_msgs/msg/Bool "{data: true}"` | Steps 1–9 execute **exactly once, in order** |
| Sequence completion | `Sequence DONE` logged; `/arm1/done` published once |
| `diff` src ↔ build ↔ install trees | All in sync after rebuild |

Full observed run after the fix:

```
Trigger received → starting pick-and-place
Step 1/9: HOME  — safe upright
Step 2/9: OPEN  — gripper open
Step 3/9: PICK  — reach to box
Step 4/9: GRIP  — close gripper
Step 5/9: CARRY — lift box
Step 6/9: PLACE — swing to bot tray
Step 7/9: DROP  — open gripper
Step 8/9: RETRACT — back to carry
Step 9/9: HOME  — return upright
Sequence DONE
```

---

## How to confirm the fix at runtime

1. Launch the sim: `ros2 launch robot_description arm_gazebo.launch.py`
   (wait for the `arm1` controllers to spawn, ~25 s).
2. In another terminal:
   ```bash
   source install/setup.bash
   ros2 run nav_nodes arm_controller_node
   ```
3. Trigger the sequence:
   ```bash
   ros2 topic pub --once /arm1/trigger std_msgs/msg/Bool "{data: true}"
   ```
4. The node log should show Steps 1–9 each exactly once, in order, followed
   by `Sequence DONE`, and the arm should perform one clean pick-and-place.


