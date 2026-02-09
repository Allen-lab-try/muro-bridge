# MURO Bridge

## Overview
**MURO Bridge** is a lightweight and modular **ROS 2 communication and coordination package** designed to support **multi-robot systems** by handling **domain bridging, topic relaying, and TF aggregation** across distributed robot instances.

The package serves as an infrastructure-level component within the **MURO Exploration Platform**, enabling scalable and reproducible **multi-robot simulation and deployment** without coupling high-level algorithms to communication details, while allowing the repository to be directly cloned and integrated with the **TurtleBot4** system for immediate use in simulation and real-robot workflows.
 

MURO Bridge is developed and maintained by the **Multi-Robot Laboratory (MURO Lab) @ UC San Diego**, as part of ongoing research on **distributed multi-agent autonomy** and **multi-robot exploration systems**.

---

## 🎥 Demonstration

### Multiple robots collaborating
The following demonstration shows multiple robots operating concurrently within the **same simulated environment**, where **MURO Bridge** enables coordinated multi-robot operation through unified communication and TF aggregation.  

All robots run in isolated ROS domains while their states, transforms, and sensor information are **bridged and visualized in a single RViz instance**, allowing system-level observation and debugging of multi-robot behavior.

<p align="center">
  <img src="docs/multi_robots.gif" width="800">
</p>

---

## 🧩 Design Philosophy
MURO Bridge is intentionally designed to be:

- **Algorithm-agnostic**  
  No assumptions are made about exploration, planning, or allocation logic.
- **Infrastructure-focused**  
  Handles *how robots talk*, not *what robots decide*.
- **Composable**  
  Can be used as a standalone ROS 2 package or integrated into larger stacks.
- **Simulation-to-Real Ready**  
  Identical communication patterns are used in simulation and real robots.

This separation allows research code (e.g., exploration or scheduling algorithms) to evolve independently from system-level communication architecture.

---

## 📦 System Structure

The diagram below illustrates the system-level architecture enabled by **MURO Bridge** in a multi-robot setting.

Each robot operates in its **own isolated ROS domain** (Domain 1 to Domain N), maintaining independent local maps and TF trees. These local TF frames and map-related information are forwarded to a dedicated **TF Aggregator** running in an intermediate domain (e.g., Domain 99), where all robot transforms are unified into a consistent global TF representation.

The unified TF information is then made available to higher-level components, such as a **centralized map merging** (e.g., Domain 100), without requiring direct coupling between individual robots. This layered design allows robots to remain fully decentralized while still supporting centralized observation, visualization, and optional global reasoning.

By separating **robot-local autonomy**, **system-level coordination**, and **cloud-level processing**, MURO Bridge provides a scalable and extensible communication backbone for multi-robot systems operating across multiple ROS domains.


<p align="center">
  <img src="docs/muro-bridge_structure.jpg" width="800">
</p>

---

## 🧱 System Context

MURO Bridge is primarily developed and tested within the **TurtleBot4** ecosystem under **ROS 2 Humble**, using **Gazebo / Ignition** for simulation.

Before using this package, please ensure that the **TurtleBot4 environment is properly installed and functional**.

> 🔗 **Reference:** [TurtleBot4 Official Documentation](https://turtlebot.github.io/turtlebot4-user-manual/)

---

## ⚙️ Quick Start

Below are example terminal commands to **launch the simulation** and **run the exploration system**.  
All commands are designed for **ROS 2 Humble** and **TurtleBot4 simulation**.


1️⃣ Source ROS and workspace environment
```bash
source /opt/ros/humble/setup.bash
source ~/muro_stack/muro_explore_platform/install/setup.bash
```
2️⃣ Launch single-robot simulation (Gazebo + RViz)
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze
```
3️⃣ Start slam module to get localization and mapping imformation
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```
4️⃣ Start navigation module to get costmap and other navigatimg imfromations
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```
5️⃣ Start frontier-based automatically exploring script
```bash 
python3 ~/muro_stack/src/mapping/frontier_extraction/script1.py
```
6️⃣ Start RViz to observe the exploration process
```bash
rviz2 
```

---
