---
layout: post
title: Emotion Display Robot Face – SAMI
description: Built the animated LCD face and ROS2 emotion-state interface for SAMI, a servo-actuated humanoid robot developed by the BAJAN Robotics team in ROB 421 Applied Robotics at Oregon State University.
skills:
  - ROS2
  - Python
  - Raspberry Pi
  - Pygame
  - Embedded Systems
  - Systems Integration

main-image: /sami_face.png
---

# Emotion Display Robot Face – SAMI

**Course:** ROB 421: Applied Robotics – Oregon State University  
**Team:** BAJAN Robotics  
**Repo:** [Martnat8/ROB421-Portfolio](https://github.com/Martnat8/ROB421-Portfolio)  
**Demo:** [@rob421_sami5 on Instagram](https://www.instagram.com/rob421_sami5/)

---

## Problem
SAMI is a servo-actuated humanoid robot capable of mirroring human motion in real time using MediaPipe pose tracking. The robot needed a face — a display system that could render expressive, animated emotions on an LCD screen mounted on the robot's head, and be triggered by other parts of the ROS2 system without tight coupling.

## What I Did

### Animated Face Display (`Eye_Test2.py`)
Developed the full Pygame-based face animation script designed to run on the Raspberry Pi driving SAMI's vertically-mounted LCD (480×800, frameless fullscreen overlay).

The animation runs at 60fps and renders:
- **Blinking eyes** — smooth lid animation on a ~3 second cycle
- **Randomized pupil shifts** — eyes drift within a ±20px range at random intervals to simulate life-like idle behavior
- **Expressive eyebrows** — rectangular blocks that track with pupil movement
- **Talking mouth** — sine-wave-driven mouth animation with occasional "talking" bursts that increase mouth height and speed, triggered by a timed state flag

The display is configured to target the Raspberry Pi's external LCD via SDL window positioning — designed from the start to run headlessly on the robot.

### ROS2 Emotion Service Interface
Designed and implemented a ROS2 service interface allowing any other node in the system to trigger emotion or talking states on the display in real time. The `talking` boolean and scheduling logic in the face script serve as the LCD-side consumer of this control signal — keeping the display fully decoupled from the motion and pose systems.

This meant the face could be updated independently by any node: the motion mirroring system, a game mode (SAMI Says), or a future interaction layer — without modifying the display code.

## Tools Used
- **ROS2** — service interface and inter-node communication
- **Pygame** — real-time animated face rendering
- **Raspberry Pi** — onboard compute driving the LCD display
- **Python** — all scripting and animation logic

---

## Demo

<blockquote class="instagram-media" data-instgrm-captioned data-instgrm-permalink="https://www.instagram.com/reel/DKsJMrdviru/?utm_source=ig_embed&utm_campaign=loading" data-instgrm-version="14" style="background:#FFF; border:0; border-radius:3px; box-shadow:0 0 1px 0 rgba(0,0,0,0.5),0 1px 10px 0 rgba(0,0,0,0.15); margin: 1px; max-width:540px; padding:0; width:99.375%;"><div style="padding:16px;"><a href="https://www.instagram.com/reel/DKsJMrdviru/?utm_source=ig_embed&utm_campaign=loading" style="background:#FFFFFF; line-height:0; padding:0 0; text-align:center; text-decoration:none; width:100%;" target="_blank">View this post on Instagram</a><p style="color:#c9c8cd; font-family:Arial,sans-serif; font-size:14px; line-height:17px; margin-bottom:0; margin-top:8px; overflow:hidden; padding:8px 0 7px; text-align:center;"><a href="https://www.instagram.com/reel/DKsJMrdviru/?utm_source=ig_embed&utm_campaign=loading" style="color:#c9c8cd; font-family:Arial,sans-serif; font-size:14px; font-style:normal; font-weight:normal; line-height:17px; text-decoration:none;" target="_blank">A post shared by SAMI BAJAN (@rob421_sami5)</a></p></div></blockquote>
<blockquote class="instagram-media" data-instgrm-captioned data-instgrm-permalink="https://www.instagram.com/reel/DKPpJPTJ3Fk/?utm_source=ig_embed&utm_campaign=loading" data-instgrm-version="14" style="background:#FFF; border:0; border-radius:3px; box-shadow:0 0 1px 0 rgba(0,0,0,0.5),0 1px 10px 0 rgba(0,0,0,0.15); margin: 1px; max-width:540px; padding:0; width:99.375%;"><div style="padding:16px;"><a href="https://www.instagram.com/reel/DKPpJPTJ3Fk/?utm_source=ig_embed&utm_campaign=loading" style="background:#FFFFFF; line-height:0; padding:0 0; text-align:center; text-decoration:none; width:100%;" target="_blank">View this post on Instagram</a><p style="color:#c9c8cd; font-family:Arial,sans-serif; font-size:14px; line-height:17px; margin-bottom:0; margin-top:8px; overflow:hidden; padding:8px 0 7px; text-align:center;"><a href="https://www.instagram.com/reel/DKPpJPTJ3Fk/?utm_source=ig_embed&utm_campaign=loading" style="color:#c9c8cd; font-family:Arial,sans-serif; font-size:14px; font-style:normal; font-weight:normal; line-height:17px; text-decoration:none;" target="_blank">A post shared by SAMI BAJAN (@rob421_sami5)</a></p></div></blockquote>
<script async src="//www.instagram.com/embed.js"></script>

*More on [@rob421_sami5](https://www.instagram.com/rob421_sami5/)*
