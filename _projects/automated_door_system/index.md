---
layout: post
order: 4
title: Automated Security Door System
description: A 3D printed miniature automated double-door system with motion detection, a handmade metal detector coil, jam detection, and a full state-machine-driven control system built on Arduino.
skills:
  - Arduino
  - C++
  - Embedded Systems
  - Sensor Integration
  - State Machine Design
  - 3D Printing
  - Electronics
  - Breadboarding

main-image: /Main.jpg
---

# Automated Security Door System

**Course:** ME 351 – Introduction to Measurement Systems  
**Team:** Walker Waggoner, Ahmed Moussaoui

---

## Problem

As delivery robots like Starship become common on campus, the question arises: how do you design a door that only lets people through — not robots? This project set out to build a miniature automated double-door system that screens who (or what) can enter based on physical characteristics of life.

The system needed to:
- Detect motion and proximity to trigger the door
- Screen for metal to identify and block robotic systems
- Never close on someone passing through
- Handle forced entry attempts and jams safely

---

## What I Did

### 3D Printing & Assembly
Modified the baseline Fusion 360 CAD to improve assembly practicality, then printed the full door structure on my Creality Surmoon D1 (~750g of filament). Spent ~4 hours on CAD modifications and another ~4 hours wiring and assembling the test article — integrating the Arduino, breadboard, servo, and all sensors into the enclosure.

### Handmade Metal Detector
Designed and built the metal detection circuit from scratch using materials I had on hand. The detector works by:
1. Pulsing a hand-wound copper coil to charge a capacitor
2. Measuring the capacitor voltage as the coil's field collapses
3. Detecting the drop in voltage that occurs when metal changes the coil's inductance

![Metal Detector Circuit](/_projects/automated_door_system/door_img-007.jpg)

To eliminate noise, I averaged **256 readings per cycle**. The baseline reading with no metal was ~0.20V — metal near the coil consistently dropped it below the threshold. No off-the-shelf module: just an inductor, capacitor, diode, and resistor.

### State Machine & Software
Wrote and debugged the bulk of the control code — a state machine with 5 states:

| State | Behavior |
|---|---|
| **Closing (A)** | Servo drives to closed; polls current for jam detection |
| **Jammed (B)** | Error state — opens door, sounds alarm, halts |
| **Closed (C)** | Idles; monitors PIR for motion; alarms on forced push |
| **Distance Check (D)** | Ultrasonic reads proximity; opens door if object < 300mm |
| **Metal Lock (E)** | Alarm sounds; door stays closed until metal clears |

Key software features:
- **Jam detection** via servo current polling — if current draw spikes above threshold while closing, the door opens and alarms instead of crushing whatever is stuck
- **Forced entry detection** — if someone pushes on the closed door, a separate alarm fires and the servo actively pushes back to closed position
- **PIR interrupt** — motion wakes the system from idle without polling
- **Toggle switch interrupt** — manual override that holds the door open regardless of state
- **Metal filtering** — 256-sample averaged analog read to cleanly distinguish metal presence from noise

### Debugging Wins
- **Servo voltage issue:** the Sunfounder 55g servo has a narrow 4.8–6V range. A single pull-down resistor dropped voltage too far, causing the servo to spin continuously. Fixed by using two 10Ω resistors in parallel (5Ω total) to keep the supply in range.
- **Ultrasonic wiring:** echo and trigger pins were swapped, causing no serial output — caught with a multimeter continuity check.

---

## Results

A fully autonomous double-door system that:
- Opens only for non-metallic objects within 300mm
- Actively blocks and alarms on metal detection
- Recovers safely from jams and forced entry
- Supports full manual override via toggle switch

---

## System Design

System schematic:

![System Schematic](/_projects/automated_door_system/door_img-000.jpg)

TinkerCad wiring diagram:

![TinkerCad Wiring](/_projects/automated_door_system/door_img-001.jpg)

State diagram:

![State Diagram](/_projects/automated_door_system/door_img-002.jpg)

---

## Build

Front — PIR sensor and ultrasonic distance sensor mounted above the door frame:

![Front of door system](/_projects/automated_door_system/door_img-004.jpg)

Back — reset button, toggle switch, and 12V power supply:

![Back of door system](/_projects/automated_door_system/door_img-005.jpg)

Underside — Arduino UNO, servo, linkages, metal detector circuit, and breadboard:

![Underside of door system](/_projects/automated_door_system/door_img-006.jpg)

---

## Demo

<iframe width="100%" height="400" src="https://www.youtube.com/embed/iyUSMOWzPmI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen style="border-radius:6px;"></iframe>
