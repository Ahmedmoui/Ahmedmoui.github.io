---
layout: post
order: 1
title: Material Sample Testing Automation
description: A ROS2-based automation system for a 7-DOF robotic manipulator designed to assist Civil Engineering lab technicians with repetitive material sample testing tasks.
skills:
  - ROS2
  - MoveIt2
  - Python
  - OpenCV
  - Docker
  - Robot Kinematics
  - Pick & Place
  - Hardware Integration

main-image: /placeholder.png
---

# Material Sample Testing Automation

## Problem
The Civil Engineering research lab at OSU relies on repetitive manual testing processes — loading material samples, initiating tests, and logging results — which consumes significant technician time and introduces human variability. The goal was to automate this workflow using a 7-DOF robotic manipulator.

## What I Did

### System Design
Led the project from inception: defined the ROS2 architecture, selected hardware, and designed the robot's behavioral pipeline to fit within the lab's existing workflow alongside active technicians.

### Pick & Place Pipeline
Built a full pick-and-place pipeline from scratch including:
- **Vision-based object detection** using OpenCV to locate sample positions
- **Grasp pose generation** to determine valid approach angles
- **MoveIt2 motion planning** for collision-aware arm trajectories

### Deployment
Dockerized the completed demo to ensure reproducible, environment-independent deployment in the lab — making handoff to lab staff straightforward.

## Results
*Assets and outcome documentation coming soon — supply images or video to complete this section.*

