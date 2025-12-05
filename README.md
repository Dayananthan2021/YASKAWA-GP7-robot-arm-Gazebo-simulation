
# YASKAWA GP7 Robot Arm Simulation (ROS 2 Jazzy + Gazebo Harmonic)

##  Introduction

This repository presents a simulation of the **YASKAWA GP7** industrial robotic arm combined with a **Robotiq 2F-85** gripper.  
Built on **ROS 2 Jazzy** and **Gazebo Harmonic**, the project focuses on executing a precise Pick-and-Place workflow: the robot retrieves a small cube (Box B) from a random pose and positions it carefully on top of a designated container (Box A) while ensuring stable orientation throughout the motion.

## 🚀 Key Capabilities

- **Accurate Robot & Gripper Modeling**  
  URDF describing the GP7 arm integrated with the Robotiq 2F-85 ensures realistic kinematics.

- **Physics-Driven Grasping**  
  The gripper interacts with objects purely through simulated friction and torque — no attach/detach hacks.

- **Inverse Kinematics Solver**  
  Levenberg–Marquardt numerical IK computed using the Peter Corke Robotics Toolbox.

- **Smooth Trajectory Generation**  
  Motion planning is based on **LSPB (trapezoidal velocity profile)** for both linear and joint-space paths.

- **Visualization & Monitoring**  
  Full support for **RViz2** frame visualization and **Gazebo Harmonic** physics simulation.

---

## System Requirements

- **Operating System:** Ubuntu 24.04 LTS (Noble)  
- **ROS 2 Distribution:** Jazzy Jalisco  
- **Simulator:** Gazebo Harmonic ≥ 8.10  
- **Python Packages:**  
  - `roboticstoolbox-python`  
  - `spatialmath-python`

---

## Setup Instructions

### 1️ Create a ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
````

### 2️ Clone the Project

```bash
git clone  gp7_sim
```

### 3️ Install Dependencies

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro
pip3 install roboticstoolbox-python spatialmath-python --break-system-packages
```

### 4️ Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

##  Running the Simulation

### **Start Gazebo with the Robot & Environment**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch gp7_sim simulation.launch.py
```

### **Start the Controller Node**

This computes kinematics and streams joint trajectory commands.

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/gp7_sim/gp7_sim/robot_controller.py
```

---

##  Technical Breakdown

###  Robot Modeling — Denavit–Hartenberg Parameters

The GP7 model is constructed using Standard D-H parameters sourced from manufacturer documentation.

| Link  | θ  | d (m) | a (m) | α (rad) | Offset               |
| ----- | -- | ----- | ----- | ------- | -------------------- |
| 1 (S) | q₁ | 0.330 | 0.040 | −π/2    | —                    |
| 2 (L) | q₂ | 0     | 0.445 | 0       | −π/2 (Vertical Home) |
| 3 (U) | q₃ | 0     | 0.040 | −π/2    | —                    |
| 4 (R) | q₄ | 0.440 | 0     | π/2     | —                    |
| 5 (B) | q₅ | 0     | 0     | −π/2    | —                    |
| 6 (T) | q₆ | 0.080 | 0     | 0       | —                    |

---

###  Pick-and-Place Task Layout

* **Box A (Target):**

  * Position: (0.3, 0.4, 0.0)
  * Dimensions: 30 × 30 × 20 cm

* **Box B (Source):**

  * Position: (0.5, −0.2, 0.0)
  * Dimensions: 5 × 5 × 5 cm

* **Grasping Sequence:**

  1. Approach Box B via trapezoidal trajectory
  2. Descend and engage fingers
  3. Lift using physics-based friction
  4. Move to the center of Box A
  5. Align and release

---

##  License

This repository is distributed under the **MIT License**. Refer to the [LICENSE](LICENSE) file for full terms.

```

```
