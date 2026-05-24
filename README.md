# RELBot Person Follower

ROS2 person-following system for the RELBot mobile robot. Runs on a Raspberry Pi 5 with a Hailo-8 NPU. Follows a target person using a yellow safety helmet (HSV) and/or person re-identification (ReID).

---

## Repository Structure

```
├── relbot_video_interface/
│   ├── relbot_video_interface/
│   │   └── video_interface_node.py     # main inference + control node
│   └── launch/
│       └── video_interface.launch.py
│
├── yolov8s_pose.hef                    # YOLO pose model
├── repvgg_a0_person_reid_512.hef       # ReID model
│
├── savePerson2Follow.py                # capture ReID embeddings of target
├── calibrate_hsv.py                    # tune HSV thresholds for helmet
└── test_detection.py                   # visualise full logic before deploying
```

All three tools share the same source flag:

| Flag | Source |
|---|---|
| *(none)* | `/dev/video0` — camera directly on Pi 5 |
| `--relbot` | UDP H264 stream from RELBot on port 5000 |

---

## Calibration — run in this order

### 1. `calibrate_hsv.py` — tune the helmet colour filter

Opens a window with the live feed and HSV mask side by side. Tune the six trackbars until the mask cleanly isolates the helmet. Press **P** to print values, **Q** to quit.

```bash
python3 calibrate_hsv.py --relbot
```

Paste the printed values into the tunable block at the top of `video_interface_node.py`:
```python
H_LOW = 18;  H_HIGH = 35
S_LOW = 100; S_HIGH = 255
V_LOW = 100; V_HIGH = 255
HELMET_MIN_PCT = 0.10
```

---

### 2. `savePerson2Follow.py` — save target ReID embeddings

```bash
python3 savePerson2Follow.py --relbot
```

1. Target stands with **back** to camera → press **Enter**
2. Target turns to face camera → press **Enter**
3. Press **Q** → saves `person_target.npz`

---

### 3. `test_detection.py` — verify logic before deploying

Runs the full state machine with live visualisation. Use this to tune parameters without rebuilding the ROS node.

```bash
python3 test_detection.py --relbot
```

**Display:**

| Element | Meaning |
|---|---|
| Grey bbox | Detected person, not target |
| Green bbox | Current target |
| Purple box | Head region |
| Cyan box | Body region (shoulder-width × YOLO height) |
| `HELMET` label | HSV filter matched |
| Body area px | Printed on body bbox — use to tune `BODY_AREA_TOO_CLOSE` |
| HUD top-left | State + Hz + lost frames + confirm streak |

---

## State Machine Logic

### Per-frame pipeline (always)

```
GET frame
RUN YOLO → persons + keypoints
FOR each person:
    GET head bbox  (head keypoints: leftmost/rightmost x, top=YOLO top, bottom=highest kp y + 25% padding)
    GET body bbox  (shoulder width × YOLO height)
    RUN HSV on head bbox → has_helmet?
```

### Shared search() function (called from all states)

```
RUN HSV on all persons
IF helmet found → return (HELMET, person)   ← skip ReID entirely
ELSE:
    RUN ReID on all persons
    IF match >= REID_THRESH → return (REID, person)
ELSE:
    return (None, None)
```

Helmet always has priority — if a helmet is found ReID is never run that frame.

### State machine

```
══════════════════════════════════════════════
SEARCHING
Motors: STOP
══════════════════════════════════════════════
run search()
IF result found:
    confirm_streak++  (same type + same person)
    IF confirm_streak >= 3:
        → switch to HELMET_FOLLOWING or PERSON_FOLLOWING
ELSE:
    confirm_streak = 0


══════════════════════════════════════════════
PERSON_FOLLOWING
Motors: FORWARD (or STOP if too close)
══════════════════════════════════════════════
run ROI ReID spatially ordered from last_cx
IF reid_match:
    insta-relock → last_cx = new x, lost_frames = 0

run HSV in background
IF helmet for 3 consecutive frames:
    → HELMET_FOLLOWING

IF no reid_match:
    lost_frames++
    IF lost_frames <= 10:
        coast forward + run search() with bias:
            REID found   → insta-relock (stay in PERSON_FOLLOWING)
            HELMET found → needs 3 confirm frames
    IF lost_frames > 10:
        → SEARCHING + stop


══════════════════════════════════════════════
HELMET_FOLLOWING
Motors: FORWARD (or STOP if too close)
══════════════════════════════════════════════
find helmet persons, order by distance from last_cx
IF helmet found:
    insta-relock → last_cx = new x, lost_frames = 0

IF no helmet:
    lost_frames++
    IF lost_frames <= 10:
        coast forward + run search() with bias:
            HELMET found → insta-relock (stay in HELMET_FOLLOWING)
            REID found   → needs 3 confirm frames → PERSON_FOLLOWING
    IF lost_frames > 10:
        → SEARCHING + stop
```

### Too close check

Every frame when a target is locked:
```
body_area = body_bbox width × height
IF body_area > BODY_AREA_TOO_CLOSE → publish stop
ELSE → publish forward
```

Use `test_detection.py` to read the body area live from the terminal and set `BODY_AREA_TOO_CLOSE` accordingly.

### Motor command published to `/object_position`

| Field | Value |
|---|---|
| `x` | target centre x scaled to `FRAME_W=320`. `-1.0` = stop |
| `y` | target centre y in raw pixels |
| `z` | `Z_FOLLOW=5000` (forward) or `Z_STOP=15000` (stop) |

---

## Tunable Parameters

All at the top of `video_interface_node.py` and `test_detection.py`.

```python
# YOLO
OBJ_THRESH = 0.50           # person detection confidence
KP_CONF    = 0.35           # keypoint visibility threshold

# Helmet HSV
H_LOW  = 18;  H_HIGH = 35
S_LOW  = 100; S_HIGH = 255
V_LOW  = 100; V_HIGH = 255
HELMET_MIN_PCT = 0.10       # fraction of head bbox that must be yellow

# ReID
REID_THRESH = 0.82          # cosine similarity threshold
ROI_PADDING = 0.10          # padding around shoulder crop

# State machine
CONFIRM_FRAMES    = 3       # consecutive frames to enter/switch a mode
LOST_FRAMES_LIMIT = 10      # frames coasting before dropping to SEARCHING

# Distance
BODY_AREA_TOO_CLOSE = 5000  # pixel area of body bbox — tune from terminal
```

---

## Running the System

**RELBot Pi — Terminal 1: stream camera**
```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video2 ! \
  image/jpeg,width=640,height=480,framerate=30/1 ! \
  jpegdec ! videoconvert ! \
  x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.0.225 port=5000
```

**RELBot Pi — Terminal 2: motor controller**
```bash
source ~/ai4r_ws/install/setup.bash && sudo ./build/demo/demo
```

**RELBot Pi — Terminal 3: sequence controller**
```bash
source ~/ai4r_ws/install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py
```

**Pi 5 — inference node**
```bash
colcon build && source install/setup.bash
ros2 launch relbot_video_interface video_interface.launch.py
```
---

## Dependencies

- ROS2 Jazzy
- `hailo_platform` Python package + Hailo-8 runtime
- OpenCV, NumPy
- GStreamer 1.0 with `gst-plugins-good`, `gst-plugins-ugly`, `gst-libav`
- Model files in `/ai4r_ws/src/`: `yolov8s_pose.hef`, `repvgg_a0_person_reid_512.hef`
