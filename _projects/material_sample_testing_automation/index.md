---
layout: post
order: 1
title: Material Sample Testing Automation
description: Ongoing automation project using a Kinova Gen3 robotic arm, ROS2, and MoveIt2 to automate repetitive material sample testing tasks in a Civil Engineering research lab at OSU.
skills:
  - ROS2
  - MoveIt2
  - Python
  - OpenCV
  - Gazebo / Ignition
  - Docker
  - Raspberry Pi
  - Pick & Place

main-image: /arm_sim_gazebo.png
---

# Material Sample Testing Automation – Project Autobeton

**Lab:** Civil Engineering Research Lab, Oregon State University  
**Robot:** Kinova Gen3 6-DOF arm with Robotiq 2F-85 gripper

---

## Problem

Civil Engineering lab technicians spend significant time on repetitive manual tasks — loading material samples into compression and shear testing machines, waiting, then logging results. Project Autobeton uses a Kinova Gen3 robotic arm to automate this workflow, freeing up technician time and reducing variability in the testing process.

Core tasks being automated:
- Pick and place of soil samples into compression test machines
- Shear and poking tests on cement samples
- Repeatable, programmable task sequences

---

## What I Did

Set up the full ROS2 + MoveIt2 software stack for the Kinova Gen3 arm from scratch — simulating the arm in Ignition Gazebo, tuning the MoveIt2 motion planner, and getting the Robotiq 2F-85 gripper working in simulation before moving to hardware.

Kinova Gen3 simulated in Ignition Gazebo with MoveIt2 path planning:

![Kinova Gen3 in Ignition Gazebo](/_projects/material_sample_testing_automation/arm_sim_gazebo.png)

MoveIt2 setup with the Gen3 6-DOF arm — exploring collision objects, path planners, and pose-to-pose planning:

![MoveIt2 arm setup](/_projects/material_sample_testing_automation/arm_moveit_setup.png)

The lab runs the arm on a dedicated portable local network (mini router), keeping the system self-contained and easy to add cameras and sensors to as the project grows.

---

## Demo

*Add robot demo video here.*

---

## Status

Active — robot arm recently returned from repairs. Next steps: vision-based sample localization with OpenCV, grasp pose generation, and a full pick-and-place pipeline to hardware.
