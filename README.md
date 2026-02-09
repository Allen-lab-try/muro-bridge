# MURO Bridge

## Overview
**MURO Bridge** is a lightweight and modular **ROS 2 communication and coordination package** designed to support **multi-robot systems** by handling **domain bridging, topic relaying, and TF aggregation** across distributed robot instances.

The package serves as an infrastructure-level component within the **MURO Exploration Platform**, enabling scalable and reproducible **multi-robot simulation and deployment** without coupling high-level algorithms to communication details.

MURO Bridge is developed and maintained by the **Multi-Robot Laboratory (MURO Lab) @ UC San Diego**, as part of ongoing research on **distributed multi-agent autonomy** and **multi-robot exploration systems**.

---

## 🎥 Demonstration

### 1. Autonomous Exploration
The following animation demonstrates the robot autonomously exploring an unknown environment using a **frontier-based exploration** strategy integrated with the **Nav2 stack**.

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

## 🧱 System Context

MURO Bridge is primarily developed and tested within the **TurtleBot4** ecosystem under **ROS 2 Humble**, using **Gazebo / Ignition** for simulation.

Before using this package, please ensure that the **TurtleBot4 environment is properly installed and functional**.

> 🔗 **Reference:** [TurtleBot4 Official Documentation](https://turtlebot.github.io/turtlebot4-user-manual/)

---

## 📦 Package Structure
