---
layout: post
order: 1
title: Material Sample Testing Automation
description: Automated material sample testing in a Civil Engineering research lab at OSU using a Kinova Gen3 robotic arm, ROS2, and computer vision — from simulation to working prototype.
skills:
  - ROS2
  - MoveIt2
  - Python
  - OpenCV
  - Gazebo / Ignition
  - Docker
  - Raspberry Pi
  - Pick & Place
  - ArUco / Pose Estimation
main-image: /arm_above_humboldt.jpg
---

# Project Autobeton — Automated Material Sample Testing

**Lab:** Civil Engineering Research Lab, Oregon State University  
**Robot:** Kinova Gen3 6-DOF arm with Robotiq 2F-85 gripper

---

<iframe width="100%" height="420" src="https://www.youtube.com/embed/cK_mL8SSwpQ" title="Project Autobeton Demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen style="border-radius:6px; margin-bottom:2rem;"></iframe>

---

## The Problem

Civil Engineering labs run hundreds of repetitive compression tests on soil and concrete samples — each one requires a technician to manually load the sample, run the machine, and log the result. Project Autobeton uses a robotic arm to handle the physical work, so researchers can focus on the science instead of the repetition.

---

## Key Decisions

### Build and test in simulation before touching the real robot

Before running any motion on the real arm, I built the full workflow in simulation — modelling the lab workspace, machine positions, and the arm's reach limits in software. This caught collision and planning issues early without risking hardware, and gave me a fast iteration loop.

<img src="/_projects/material_sample_testing_automation/rviz_annotated.png" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

### Use visual reference markers instead of fixed positions

Hard-coding coordinates breaks any time a table gets nudged. Instead, I placed ArUco markers — small printed reference patterns, similar to QR codes — on each testing machine. The robot reads these with a camera before every run and computes exactly where the machine is. This made the system robust to the real-world variability of a shared lab space.

<img src="/_projects/material_sample_testing_automation/humboldt_annotated.png" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

### Build a vision pipeline to detect samples by shape

Rather than manually placing each sample in a fixed spot, I built a computer vision pipeline that locates soil sample cylinders on the workspace table by their shape and size. The robot uses this to plan its pick — removing a manual step and bringing the system closer to fully autonomous operation.

<img src="/_projects/material_sample_testing_automation/perception_annotated.png" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

### Keep the system self-contained and portable

The robot, camera, and control laptop all run on a dedicated mini router that travels with the setup. No dependency on lab infrastructure means the system can be repositioned anywhere in the building, and adding new sensors is as simple as joining the network.

---

## The Prototype

<img src="/_projects/material_sample_testing_automation/lab_setup_overview.jpg" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1rem;">
<img src="/_projects/material_sample_testing_automation/arm_above_humboldt.jpg" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1rem;">
<img src="/_projects/material_sample_testing_automation/gripper_closeup.jpg" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

---

## Impact

- **Frees technician time** — compression tests that required constant manual attention can now run with the robot handling the physical steps
- **More consistent results** — the robot places samples the same way every time, removing a source of human variability from the testing process
- **Built to expand** — the modular architecture makes it straightforward to add new machines, sensors, or test types as the lab's needs evolve

---

**Status:** Active — full pick-and-place pipeline working on hardware. Next: multi-sample autonomous test runs.
