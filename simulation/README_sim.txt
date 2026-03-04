# lbr_utils - CSV IK Mission Control

## 1. Description

This ROS 2 Python node (`csv_ik_node_final`) controls a robotic arm (KUKA LBR) by processing pose data from a CSV file. It calculates the target position of an object seen by a camera and executes a precise, multi-stage movement sequence using MoveIt 2.

### Key Features

* **Coordinate Transformation:** Converts camera-space coordinates ($Xf, Yf, Zf$) into the robot's base frame using a calibrated Hand-Eye matrix ($T_{cam\_to\_ee}$).
* **Smooth Motion Control:** * Uses `max_step` of 2mm for high-resolution Cartesian paths.
* Implements time scaling (slowdown) to ensure fluid, jitter-free descent.


* **RViz Visual Feedback:** Publishes Markers to show the calculated target (Yellow Sphere) and the workspace (Red Box).
* **Interactive Selection:** Allows the user to choose which specific capture to execute from the CSV list.

### Mission Phases

1. **Initial Pose:** Moves the arm to the exact location where the data was originally captured.
2. **XY Alignment:** Shifts the End Effector horizontally to be directly above the target.
3. **Smooth Descent:** Performs a slow vertical move towards the target, stopping at a defined `safe_height`.

### Requirements

* **Environment:** ROS 2 (Humble or newer), MoveIt 2.
* **Robot:** LBR Stack (compatible with `/lbr/` action servers).
* **Python Deps:** `numpy`, `rclpy`, `geometry_msgs`, `moveit_msgs`, `visualization_msgs`.
* **Data File:** A CSV file located at: `/home/user/lbr-stack/src/lbr_utils/data/captures.csv`

### CSV Structure

The node expects the following columns:

* `px, py, pz`: End Effector position during capture.
* `qx, qy, qz, qw`: End Effector orientation (quaternion).
* `Xf, Yf, Zf`: Relative coordinates of the object from the camera.

---

## 2. Installation & Setup

Follow these steps to integrate the script into a formal ROS 2 package structure.

### 1. Create the Workspace Structure

```bash
cd ~/lbr-stack/src
ros2 pkg create lbr_utils --build-type ament_python

```

### 2. Organize Files

Ensure your directory looks like this:

```text
lbr-stack/src/lbr_utils/
├── lbr_utils/
│   ├── __init__.py
│   └── csv_ik_node_final.py      <-- Place your script here
├── data/
│   └── captures.csv              <-- Place your data here
├── package.xml
├── setup.py
└── setup.cfg

```

### 3. Permissions & Dependencies

Make the script executable:

```bash
chmod +x ~/lbr-stack/src/lbr_utils/lbr_utils/csv_ik_node_final.py

```

Edit **package.xml** and add:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>moveit_msgs</exec_depend>
<exec_depend>visualization_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>

```

### 4. Configure Entry Points

Edit **setup.py**:

```python
entry_points={
    'console_scripts': [
        'csv_ik_node = lbr_utils.csv_ik_node_final:main',
    ],
},

```

### 5. Build and Source

```bash
cd ~/lbr-stack
colcon build --packages-select lbr_utils
source install/setup.bash

```

---

## 3. Execution (How to Run)

Open a new terminal for each step and remember to source the workspace.

**Terminal 1 — Launch Mock Robot**

```bash
ros2 launch lbr_bringup mock.launch.py model:=iiwa7

```

**Terminal 2 — Start MoveGroup (IK Engine)**

```bash
ros2 launch lbr_bringup move_group.launch.py mode:=mock model:=iiwa7 rviz:=false

```

**Terminal 3 — Launch RViz**

```bash
ros2 launch lbr_bringup rviz.launch.py model:=iiwa7

```

**Terminal 4 — Run the IK Node**

```bash
ros2 run lbr_utils csv_ik_node

```

---

## 4. Visualization (RViz)

To see the target and the workspace:

1. In **Global Options**, set `Fixed Frame` to `lbr_link_0`.
2. Click **Add** -> **Marker**.
3. In the Marker properties, set the Topic to: `/visualization_marker`.

**Visual Legend:**

* 🟡 **Yellow Sphere:** Target centroid.
* 🔴 **Red Box:** Target object workspace.

