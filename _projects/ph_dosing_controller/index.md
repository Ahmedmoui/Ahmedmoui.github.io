---
layout: post
order: 6
title: pH Dosing Controller
description: Built an automated pH monitoring and dosing system using a Raspberry Pi 4, Atlas Scientific EZO-pH sensor, and dual EZO-PMP peristaltic pumps to hold a liquid at a target pH setpoint.
skills:
  - Raspberry Pi
  - Python
  - Flask
  - REST APIs
  - I2C / Hardware Drivers
  - Control Systems
  - Atlas Scientific EZO Sensors
  - Tailscale VPN / Remote Access

main-image: /rig_hero.jpg
---

# pH Dosing Controller

**Repo:** [Ahmedmoui/ph-controller](https://github.com/Ahmedmoui/ph-controller)

---

## Overview
An automated pH control system that continuously monitors the pH of a liquid and doses acid or base as needed to hold it at a target setpoint, instead of relying on manual titration.

<img src="/_projects/ph_dosing_controller/rig_hero.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

## Impact
This controller enables more complex and tightly controlled experiments in the civil engineering labs — studying the effects of pH on cement, soil, and other materials — by holding pH steady automatically instead of requiring someone to titrate by hand throughout a run.

<img src="/_projects/ph_dosing_controller/dosing_curve.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

## Hardware
- **Raspberry Pi 4** — onboard compute running the controller and web backend
- **Atlas Scientific EZO-pH circuit** — pH measurement
- **Two Atlas Scientific EZO-PMP peristaltic pumps** — acid and base dosing
- **i3 Interlink** — electrical isolation between the Pi and pump/sensor circuits
- **Hosyond capacitive touchscreen** — local live readout and control

<img src="/_projects/ph_dosing_controller/pump_closeup.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

## Software
Built around a Flask web backend with a modular structure:
- **Core controller** runs a background dosing loop that doses within a configurable deadband around the setpoint
- **Hardware drivers** handle low-level I2C communication with the EZO devices
- **REST API** for remote control and integration with other tools/scripts
- **Web dashboard** for live pH charting, manual pump priming/control, and calibration wizards
- **Session logging** to CSV for after-the-fact analysis
- **Tailscale VPN + HTTP Basic Auth** for secure remote access to the dashboard

<img src="/_projects/ph_dosing_controller/build_mounting.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

### Web Dashboard
Live pH readout against a target setpoint, manual pump control, and a real-time chart of pH and dosing volume over time.

<img src="/_projects/ph_dosing_controller/ui_dashboard.png" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

I2C addressing, dosing parameters (deadband, dose per cycle, poll interval), and a guided pH probe calibration wizard (mid/low/high point) are all configurable from the dashboard.

<img src="/_projects/ph_dosing_controller/ui_config.png" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">
