---
layout: post
title: Autonomous RC Aircraft – DBF Capstone
description: Led the autonomous systems sub-team for Oregon State's Design Build Fly capstone, achieving the team's first successful autonomous test flight using an ArduPilot-based autopilot.
skills:
  - ArduPilot
  - Mission Planner
  - Avionics Integration
  - Soldering
  - Wiring & Electrical Assembly
  - Autonomous Systems
  - Systems Integration
  - Flight Testing

main-image: /DBF ahmed frame cropped.jpeg
---

# Autonomous RC Aircraft – Design Build Fly Capstone

**Competition:** AIAA Design, Build, Fly (DBF) 2024/25  
**Sponsor:** OSGC, AIAA  
**Report:** [DBF Final Report](/DBF FINAL REPORT.docx.pdf)

---

## Problem
The AIAA DBF competition challenged our team to design and build a custom RC aircraft capable of captive-carry and autonomous flight — modeling the real-world X-1 supersonic aircraft experiments. Midway through the project, the team narrowly missed the competition cutoff. Rather than stopping, the scope was redefined: remove the glider, and make the mothership fully autonomous.

The core challenge became: validate a complete ArduPilot-based autopilot stack on a custom-built aircraft and achieve autonomous flight — a first for the OSU DBF team.

## What I Did

### Autonomous Systems Lead
Led the autonomous sub-team responsible for selecting, configuring, and validating the entire autopilot stack:
- Configured **ArduPilot** firmware for the aircraft's flight dynamics and mission profiles
- Used **Mission Planner** for avionics setup, pre-flight checks, and live telemetry monitoring
- Defined and tested autonomous waypoint missions and failsafes (low battery, lost telemetry, recovery modes)

### De-risking with a Test Bed
Before integrating into the custom airframe, I validated the full autopilot configuration on a retrofitted off-the-shelf aircraft. This meant any issues with ArduPilot tuning or Mission Planner configuration were caught on a sacrificial platform — not the hand-built competition plane.

### Solving a Manufacturing Problem
During wing assembly, the team discovered an unintended geometric twist at the wingtips causing a negative angle of attack — a condition that would prevent the aircraft from generating lift at takeoff. The fix: switch the landing gear configuration from tricycle to **taildragger**, lowering the tail to restore a positive AoA on the ground without touching the wing structure.

### Hardware Fabrication
Hands-on build throughout:
- Wing frame fabrication (balsa and basswood ribs, carbon fiber spar)
- Full avionics wiring and soldering — flight controller, ESCs, receivers, power distribution
- Airframe assembly and systems integration

## Results
Achieved the team's **first successful autonomous test flight**, validating the ArduPilot integration, waypoint navigation, and autonomous landing capability on the custom-built aircraft.

---

## Build Process

Wing frame fabrication:

<video controls width="100%" style="border-radius:6px; margin-bottom:1rem;">
  <source src="/IMG_5198.mov" type="video/quicktime">
  Your browser does not support the video tag.
</video>

First thrust test with props mounted:

<video controls width="100%" style="border-radius:6px; margin-bottom:1rem;">
  <source src="/IMG_5369.mov" type="video/quicktime">
  Your browser does not support the video tag.
</video>

Control surfaces bench test:

<video controls width="100%" style="border-radius:6px; margin-bottom:1rem;">
  <source src="/20250602_105457.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
