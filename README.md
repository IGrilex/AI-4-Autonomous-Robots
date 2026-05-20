# RELBot Person Follower

A ROS2-based person-following system for the RELBot mobile robot, running on a Raspberry Pi 5 with a Hailo-8 NPU. The robot identifies and follows a target person using a combination of yellow safety helmet detection (HSV colour filter) and person re-identification (ReID).

---

## Task

The RELBot receives a live H264 video stream from a camera mounted on the robot. A Raspberry Pi 5 runs two Hailo-accelerated neural networks on every frame:

- **YOLOv8s-Pose** — detects all people in the scene and their body keypoints
- **RepVGG-A0 ReID** — produces a 512-dimensional appearance embedding for a person crop

The system follows one specific person according to a priority rule:

> **If the target is wearing a yellow safety helmet → follow by helmet.**
> **If the helmet is not visible → follow by saved ReID embedding.**

Motor commands are published as a ROS2 `geometry_msgs/Point` message to `/object_position`, which the sequence controller reads to steer and drive the robot.

---

## Repository Structure

```
├── video_interface_node.py      # Main inference + control node (runs on Pi 5)
├── video_interface.launch.py    # ROS2 launch file for the node
├── savePerson2Follow.py         # Calibration: capture ReID embeddings of target
├── calibrate_hsv.py             # Calibration: tune HSV thresholds for helmet colour
├── test_detection.py            # Calibration: full visualisation of the 3-state logic
└── README.md
```

---

## Calibration Tools

All three tools share the same source flag:

| Flag | Source |
|---|---|
| *(none)* | `/dev/video0` — camera directly connected to the Pi 5 |
| `--relbot` | UDP H264 stream from the RELBot on port 5000 |

---

### 1 — `calibrate_hsv.py` — Tune the helmet colour filter

Opens a single window with two panels side by side:
- **Left** — live camera feed with current H/S/V values and match percentage overlaid
- **Right** — binary mask: **white = passes filter**, black = rejected

Six trackbars let you tune the HSV range live. When the mask cleanly isolates the helmet and nothing else, press **P** to print the values ready to paste into `video_interface_node.py`.

```bash
# Using direct camera
python3 calibrate_hsv.py

# Using RELBot stream
python3 calibrate_hsv.py --relbot
```

**Controls:** trackbars for `H low / S low / V low / H high / S high / V high` · **P** = print values · **Q** = quit

Paste the output into the tunable block at the top of `video_interface_node.py`:

```python
H_LOW  = 20;  H_HIGH = 35
S_LOW  = 100; S_HIGH = 255
V_LOW  = 100; V_HIGH = 255
HELMET_MIN_PCT = 0.25
```

---

### 2 — `savePerson2Follow.py` — Save the target person's ReID embeddings

Captures two appearance embeddings of the target person (back view and front view) and saves them to `/ai4r_ws/src/person_target.npz`. This file is loaded by the inference node at startup.

```bash
# Using direct camera
python3 savePerson2Follow.py

# Using RELBot stream
python3 savePerson2Follow.py --relbot
```

**Workflow:**
1. Target person stands with their **back** to the camera
2. Press **Enter** — back embedding saved
3. Target person turns to **face** the camera
4. Press **Enter** — front embedding saved
5. Press **Q** — saves `person_target.npz` and exits

The terminal prints the cosine similarity between the two embeddings as a sanity check. Values above `0.4` are normal; very high values may mean front and back look too similar for ReID to be reliable.

---

### 3 — `test_detection.py` — Visualise the full 3-state logic

Runs the complete state machine (YOLO + HSV + ReID) with the same parameters as the node, but displays everything visually so you can verify behaviour before deploying.

```bash
# Using direct camera
python3 test_detection.py

# Using RELBot stream
python3 test_detection.py --relbot
```

**Display:**

| Element | Meaning |
|---|---|
| Grey bounding box | Detected person, not the target |
| Green bounding box | Current followed target |
| Purple box on head | Head region (from YOLO top + visible head keypoints) |
| `HELMET` label | Yellow filter fired on that person's head region |
| Top-left HUD | Current state + loop frequency in Hz |

**States shown in HUD:**

| State | What the robot is doing |
|---|---|
| `SEARCHING` | No target locked. Checking for helmet every frame, running ReID every 5 frames |
| `HELMET_FOLLOWING` | Following the person with the yellow helmet |
| `PERSON_FOLLOWING` | Helmet not visible; following by ReID embedding |

Press **Q** to quit.

---

## Node Logic — `video_interface_node.py`

### Overview

The node subscribes to the GStreamer frame source and runs the following pipeline on every frame:

```
Frame → YOLO pose → per-person head bbox → HSV helmet check
                                         → 3-state machine → /object_position
```

YOLO and HSV run **every frame**. ReID only runs when needed (see states below).

### Head Bounding Box

For each detected person, a head region is computed from the visible head keypoints (nose, eyes, ears):

```
top    = y1 of the YOLO person bounding box
left   = min X of visible head keypoints
right  = max X of visible head keypoints
bottom = max Y of visible head keypoints
```

The helmet check (HSV filter) runs only on this crop, not the full frame — keeping it fast.

### Helmet Detection

The head crop is converted to HSV. The fraction of pixels inside the configured colour range is computed. If it exceeds `HELMET_MIN_PCT` (default 25 %), the person is considered to be wearing the helmet.

### 3-State Machine

```
         helmet found (any frame)
SEARCHING ──────────────────────► HELMET_FOLLOWING
    │                                    │
    │ ReID match (every 5th frame)       │ helmet lost > MAX_LOST_FRAMES frames
    ▼                                    ▼
PERSON_FOLLOWING ◄──────────── SEARCHING
    │   helmet reappears
    └──────────────────────────► HELMET_FOLLOWING
```

**`SEARCHING`**
- HSV check every frame — switches to `HELMET_FOLLOWING` immediately on first hit
- ReID runs every `REID_EVERY_N` frames on all detected persons — switches to `PERSON_FOLLOWING` on first match above `REID_THRESH`
- Publishes stop command while no target is found

**`HELMET_FOLLOWING`**
- Follows whichever person the helmet filter fires on
- No ReID is run — helmet detection alone is sufficient
- If helmet is lost for more than `MAX_LOST_FRAMES` consecutive frames → `SEARCHING`
- No saved ReID embedding for the helmet person is ever made; the switch to `SEARCHING` is intentional

**`PERSON_FOLLOWING`**
- HSV check every frame — if helmet reappears, switches to `HELMET_FOLLOWING` immediately
- ReID runs every frame on all detected persons, sorted by Manhattan distance from the last known centre — stops at the first match, so if the person is still nearby it is found in one ReID call
- If no ReID match for more than `MAX_LOST_FRAMES` frames → `SEARCHING`

### Motor Command

The node publishes `geometry_msgs/Point` to `/object_position`:

| Field | Value |
|---|---|
| `x` | Lateral centre of the target, scaled to `FRAME_W=320`. `−1.0` means stop |
| `y` | Vertical centre of the target in raw pixels |
| `z` | Bounding-box area × `AREA_SCALE`, smoothed with EMA |

Centre and area are computed from the **shoulder keypoints** when visible (more stable than the full bounding box), falling back to the YOLO box otherwise. Both the lateral error and the area are smoothed with an exponential moving average before publishing to avoid jerky motor commands.

### Tunable Parameters

All parameters are at the top of `video_interface_node.py` — nothing else needs to be edited.

```python
# YOLO
OBJ_THRESH = 0.50      # person detection confidence
KP_CONF    = 0.35      # keypoint visibility threshold

# Helmet HSV
H_LOW  = 20;  H_HIGH = 35     # hue range
S_LOW  = 100; S_HIGH = 255    # saturation range
V_LOW  = 100; V_HIGH = 255    # value range
HELMET_MIN_PCT = 0.25          # fraction of head bbox that must be yellow

# ReID
REID_THRESH  = 0.82    # cosine similarity threshold
REID_EVERY_N = 5       # ReID cadence in SEARCHING state

# State machine
MAX_LOST_FRAMES = 5    # grace frames before state change

# Motor control
AREA_SCALE  = 0.1      # scales raw area into z value
AREA_ALPHA  = 0.2      # EMA factor for area smoothing
ERROR_ALPHA = 0.2      # EMA factor for lateral error smoothing

# Debug
SHOW_VIEWER = False    # set True to open a CV2 window on the Pi 5
```

---

## Running the System

### RELBot Pi — Terminal 1: stream the camera

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video2 ! \
  image/jpeg,width=640,height=480,framerate=30/1 ! \
  jpegdec ! videoconvert ! \
  x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.0.225 port=5000
```

### RELBot Pi — Terminal 2: motor controller demo

```bash
source ~/ai4r_ws/install/setup.bash
cd ~/ai4r_ws/
sudo ./build/demo/demo
```

### RELBot Pi — Terminal 3: sequence controller

```bash
source ~/ai4r_ws/install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py
```

### Pi 5 — inference node

```bash
colcon build
source install/setup.bash
ros2 launch relbot_video_interface video_interface.launch.py
```

---

## Dependencies

- ROS2 (tested on Humble)
- `hailo_platform` Python package with Hailo-8 runtime
- OpenCV (`cv2`)
- NumPy
- GStreamer 1.0 with `gst-plugins-good`, `gst-plugins-ugly`, `gst-libav`
- Model files (`.hef`) in `/ai4r_ws/src/`:
  - `yolov8s_pose.hef`
  - `repvgg_a0_person_reid_512.hef`
