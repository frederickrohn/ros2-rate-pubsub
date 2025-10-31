# rate_pubsub

A parameter-driven ROS 2 publisher/subscriber system implemented in both **C++** and **Python**, designed for flexible, real-time communication.

---

## 🚀 Overview

This package demonstrates a **dynamic publish/subscribe architecture** built with **ROS 2 Jazzy** on **Ubuntu 24.04**.  
It enables **runtime reconfiguration of publish rates** through ROS 2 parameters, allowing you to adjust message frequency without restarting nodes.

Key technical highlights:
- **Parameter-based control:** Adjust publisher rate on the fly using ROS 2 parameters.
- **Thread safety:** Uses mutex-protected timer rebuilding to avoid race conditions between parameter callbacks and active publisher loops.
- **Cross-platform validation:** Developed on a VM and deployed successfully on a **Raspberry Pi 5** running Linux for embedded real-time testing.

---

## 🧩 Dependencies

- ROS 2 Humble (or later)
- C++17 compiler
- `rclcpp`, and standard ROS 2 message libraries

---

## 🛠️ Build Instructions

```bash
cd ~/Workspaces/ros2_snippets_ws
colcon build --packages-select rate_pubsub
source install/setup.bash
```
---

## ▶️ Run the Nodes

C++ Version
ros2 run rate_pubsub rate_listener
ros2 run rate_pubsub rate_talker

---

## ⚙️ Change Publish Rate at Runtime

You can update the publishing frequency while the node is running:

ros2 param set /rate_talker publish_hz 2.0

---

## 🧠 Notes

This project bridges the gap between desktop simulation and embedded deployment, providing a clean reference for building responsive ROS 2 applications that support runtime parameter updates.
