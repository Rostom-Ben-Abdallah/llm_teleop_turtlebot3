````markdown
# LLM Teleop + YOLO Scene Understanding for TurtleBot3 (ROS 2 Humble)

This project lets you:

- Control a **TurtleBot3** robot in Gazebo using **natural language** via an LLM (Ollama + Llama 3.1).
- Run a **YOLO-based scene node** that uses a camera (webcam or ROS topic) and:
  - detects objects with YOLO
  - returns a JSON summary via `/describe_scene`
  - explains the scene in natural language via `/explain_scene` (using the LLM)

Everything is built in a ROS 2 workspace called `ros2_ws`.

---

## 0. System Requirements

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble (desktop installation with Gazebo)
- **GPU**: Optional (YOLO can run on CPU)
- **Git + Python3** installed

---

## 1. One-time Installation

Do these steps once on a new machine.

### 1.1 Install TurtleBot3 + Gazebo + RViz

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros \
  ros-humble-rviz2
````

Set TurtleBot3 model (you can also add this to your `~/.bashrc`):

```bash
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
export TURTLEBOT3_MODEL=waffle_pi
```

### 1.2 Python dependencies

Use user-level `pip` (no sudo):

```bash
pip3 install --user ultralytics opencv-python requests numpy
```

### 1.3 Install Ollama + Llama 3.1

1. Install **Ollama** from the official website (Linux installer).
2. Pull the model:

```bash
ollama pull llama3.1:8b
```

3. Check that the service is running and the model exists:

```bash
curl -s http://127.0.0.1:11434/api/version && echo
curl -s http://127.0.0.1:11434/api/tags | python3 -m json.tool
```

You should see something like:

```json
"models": [
  {
    "name": "llama3.1:8b",
    ...
  }
]
```

If not, fix Ollama before continuing.

---

## 2. Clone This Project and Build the Workspace

We use a standard ROS 2 workspace called `ros2_ws`.

### 2.1 Create workspace and clone

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone repo into the workspace root
git clone git@github.com:Rostom-Ben-Abdallah/llm_teleop_turtlebot3.git .

# (If you prefer HTTPS instead of SSH, use:)
# git clone https://github.com/Rostom-Ben-Abdallah/llm_teleop_turtlebot3.git .
```

Now your tree looks like:

```text
~/ros2_ws/
├── src/
│   └── llm_teleop/
│       ├── package.xml
│       ├── setup.py
│       └── llm_teleop/
│           ├── __init__.py
│           ├── llm_node.py        # LLM → /cmd_vel node
│           └── yolo_scene_node.py # YOLO + /describe_scene + /explain_scene
├── README.md
├── build/
├── install/
└── log/
```

### 2.2 Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --merge-install --symlink-install
```

### 2.3 (Optional) Auto-source the workspace

Add this to the **end** of `~/.bashrc`:

```bash
# ROS 2 Humble + workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=0
```

Then reload:

```bash
source ~/.bashrc
```

---

## 3. How to Run the Full Demo (Terminal by Terminal)

We’ll assume **ROS_DOMAIN_ID=0** for everything.

### Terminal 1 – Start TurtleBot3 Simulation in Gazebo

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This will:

* start `gzserver` and `gzclient` (Gazebo GUI)
* spawn a TurtleBot3 Waffle Pi in the world
* expose `/cmd_vel` to control the robot

Leave this terminal running.

---

### Terminal 2 – LLM Teleop Node (Ollama → /cmd_vel)

This node sends your natural language prompt to **Ollama**, receives a JSON like:

```json
{
  "linear": 0.1,
  "angular": 0.0,
  "duration": 2.0
}
```

then publishes `Twist` commands on `/cmd_vel`.

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Example: go forward slowly for 2 seconds
ros2 run llm_teleop llm_teleop \
  --ros-args -p prompt:="go forward slowly for 2 seconds"
```

What you should see:

* In this terminal: logs like
  `Calling Ollama /api/chat with model 'llama3.1:8b'...`
  `Executing motion: linear=0.100 m/s, angular=0.000 rad/s, duration=2.00 s`
* In Gazebo: robot moves forward a bit then stops.

Try more prompts:

```bash
ros2 run llm_teleop llm_teleop --ros-args -p prompt:="rotate left in place"
ros2 run llm_teleop llm_teleop --ros-args -p prompt:="go forward and turn slightly right"
```

If Ollama fails or JSON is invalid, the node falls back to:

* `linear = 0.2 m/s`
* `angular = 0.0`
* `duration = 2.0 s`

---

### Terminal 3 – YOLO Scene Node (Object Detection + Services)

This node grabs a frame from a camera (webcam or configured source), runs YOLO, and exposes:

* `/describe_scene` – returns raw detections JSON
* `/explain_scene` – calls LLM and returns a natural language description

> By default we use YOLOv8n. Make sure you have a model file, e.g. `yolov8n.pt` (Ultralytics will auto-download if needed).

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run llm_teleop yolo_scene \
  --ros-args \
    -p model_path:="yolov8n.pt" \
    -p conf:=0.4 \
    -p device:="cpu"
```

Leave this running.

#### 3.1 Test `/describe_scene`

Open **Terminal 4**:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /describe_scene std_srvs/srv/Trigger {}
```

Expected style of response (example):

```text
response:
std_srvs.srv.Trigger_Response(
  success=True,
  message='{
    "summary": "Detected: person (0.47), person (0.40)",
    "objects": [
      {
        "label": "person",
        "conf": 0.4700,
        "x1": 2,
        "y1": 77,
        "x2": 192,
        "y2": 239
      },
      {
        "label": "person",
        "conf": 0.4025,
        "x1": 94,
        "y1": 79,
        "x2": 188,
        "y2": 233
      }
    ],
    "ts": 1763942839.20
  }'
)
```

#### 3.2 Test `/explain_scene` (YOLO + LLM)

Still in Terminal 4:

```bash
ros2 service call /explain_scene std_srvs/srv/Trigger {}
```

Expected: a more “human” explanation, e.g.:

> “There are two people standing close together in front of the camera. No other objects are clearly visible.”

---

### Terminal 4 (Optional) – RViz Visualization

If you want a nicer visualization than Gazebo only:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash

ros2 run rviz2 rviz2
```

In RViz:

1. Set **Fixed Frame** to `odom`
2. Add:

   * `RobotModel` (Robot Description)
   * `LaserScan` on `/scan`
   * `Odometry` on `/odom`
3. Now when you run `llm_teleop` in Terminal 2, you will see the robot moving in RViz+Gazebo.

---

## 4. Quick Checklist for Your Friend After `git clone`

After cloning and before trying the demo, your friend should:

1. Install ROS 2 Humble + TurtleBot3 + Gazebo + RViz

2. Install Python deps:

   ```bash
   pip3 install --user ultralytics opencv-python requests numpy
   ```

3. Install Ollama and pull `llama3.1:8b`:

   ```bash
   ollama pull llama3.1:8b
   ```

4. Build the workspace:

   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --merge-install --symlink-install
   ```

5. Run the 3 main terminals in this order:

   * **Terminal 1**: `turtlebot3_world.launch.py`
   * **Terminal 2**: `ros2 run llm_teleop llm_teleop --ros-args -p prompt:="go forward slowly for 2 seconds"`
   * **Terminal 3**: `ros2 run llm_teleop yolo_scene ...` and test `/describe_scene` + `/explain_scene`.

If all of that works, the project is correctly installed and running ✅

---

## 5. Notes

* This project is meant as a **demo / playground** for:

  * LLM → low-level robot velocity commands
  * YOLO → perception + LLM description
* It is easy to extend:

  * different robot models (changing `/cmd_vel` target)
  * different YOLO models (`yolov8s.pt`, `yolov8m.pt`, etc.)
  * more complex prompts (“avoid obstacles”, “rotate until you see a person”, etc.)

Enjoy! 😄

```


