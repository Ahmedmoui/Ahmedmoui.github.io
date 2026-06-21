---
layout: post
order: 5
title: Popsicle-Stick Ferris Wheel Build Kits
description: Designed and 3D printed a 35-kit batch of snap-together joints that let kids build their own working popsicle-stick Ferris wheel, commissioned by a local child development center.
skills:
  - Fusion 360
  - Parametric Modeling
  - 3D Printing
  - PrusaSlicer
  - Design for Manufacturing
  - Trigonometry / Geometric Design
  - Batch Production
main-image: /gym_display_wheels_crop.jpg
---

# Popsicle-Stick Ferris Wheel Build Kits

**Client:** A local child development center
**Deliverable:** 35 build-your-own-Ferris-wheel kits for a kids' craft activity

---

## The Brief

A local child development center wanted a hands-on craft activity kids could build themselves: a Ferris wheel made from ordinary popsicle sticks, easy enough for small hands to assemble on their own. Their first prototype was straws and pieces of paper held together with glue — functional as a one-off, but too fragile and fiddly to hand to 35 different kids. I proposed a 3D printed alternative instead: a set of joints that snap onto sticks and hold them at the exact angles needed to form a hexagonal wheel, repeated across 35 kits.

<video src="/_projects/little_gym_ferris_wheel_kits/assembly_clip.mp4" autoplay loop muted playsinline style="max-width:420px; width:100%; border-radius:6px; margin-bottom:1.5rem;"></video>

## Working Out the Geometry

A popsicle-stick Ferris wheel is really just two hexagonal trusses connected by a center hub, so the whole design came down to getting a handful of joint angles right. I worked the geometry by hand first — law of sines to pin down the triangle formed by each spoke and rim segment — before touching CAD.

<img src="/_projects/little_gym_ferris_wheel_kits/sketch_center_joint_calc.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

That math turned into a parts catalog: a 2x center joint, a 6x side joint, and a 2x leg stand, each with its own angle and stick-pocket dimensions sized to a standard popsicle stick.

<img src="/_projects/little_gym_ferris_wheel_kits/sketch_joint_catalog.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

## Modeling and Printing

I modeled the joint family parametrically in **Fusion 360** so the stick-pocket width, wall thickness, and hub angle were all driven off a few variables — useful once I started iterating on fit (the first pass was too tight for a 6-year-old's hands to assemble without snapping a stick).

<img src="/_projects/little_gym_ferris_wheel_kits/cad_workstation.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

Each kit needed 10 joints, so the bulk of the work was production printing — nesting parts on the bed, slicing in **PrusaSlicer**, and running a Creality FDM printer through dozens of plates to get to 35 complete kits' worth of hardware.

<img src="/_projects/little_gym_ferris_wheel_kits/printer_bed_joints.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

Not every plate finished clean — long unattended runs occasionally ended in a tangle of stringing instead of parts:

<img src="/_projects/little_gym_ferris_wheel_kits/printer_spaghetti.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

## Prototyping the Assembly

Before locking in dimensions, I mocked up wheels at the kitchen table with loose sticks and an early joint set, checking that a hexagonal rim actually closes up cleanly and that the hub holds the cross-spokes at the right tension.

<img src="/_projects/little_gym_ferris_wheel_kits/hub_unassembled.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">
<img src="/_projects/little_gym_ferris_wheel_kits/stick_hexagon_experiment.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">
<img src="/_projects/little_gym_ferris_wheel_kits/wheel_with_sticks.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

I also ran a batch in a glow-in-the-dark filament, which doubled as an easy way to spot fit issues against the light:

<img src="/_projects/little_gym_ferris_wheel_kits/green_wheel_closeup.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">
<img src="/_projects/little_gym_ferris_wheel_kits/green_wheel_backlit.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

## The Finished Kits

<img src="/_projects/little_gym_ferris_wheel_kits/packed_kits.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">
<img src="/_projects/little_gym_ferris_wheel_kits/product_shot_a.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">
<img src="/_projects/little_gym_ferris_wheel_kits/product_shot_b.jpg" style="max-width:550px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

35 kits shipped out and went up on display at the center:

<img src="/_projects/little_gym_ferris_wheel_kits/gym_display.jpg" style="max-width:700px; width:100%; border-radius:6px; margin-bottom:1.5rem;">

---

**Status:** Complete — 35 kits delivered and installed.
