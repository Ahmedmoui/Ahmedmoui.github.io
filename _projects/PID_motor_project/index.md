---
layout: post  
title: PID Motor Velocity Controller  
description: A comprehensive guide on designing a PID controller for precise motor velocity control, leveraging real-time feedback and tuning for optimal performance.  
skills:  
  - Control Algorithms  
  - Motor Control  
  - Embedded Systems  
  - Arduino Programming  
  - Feedback Systems  
  - Systems Integration  

main-image: /images/pid_motor_velocity.jpg  
---

# PID Motor Velocity Controller  

## Introduction  
This project focuses on designing a closed-loop feedback system for controlling motor velocity using a PID controller. The aim is to develop a controller capable of maintaining an accurate motor speed, adjusting dynamically based on real-time feedback from an encoder. By the end of the project, the controller should be able to precisely regulate the motor’s output shaft velocity, making it adaptable to applications such as robotics and automation.

## Implication  
PID controllers are widely used in various industries where precise control over system parameters like speed, temperature, or position is required. In this case, the PID motor controller allows for accurate velocity control in motors, which is essential in applications ranging from robotic arms to conveyor belts and drones.

## Setup & Design  
### Hardware  

The system consists of a TS-25GA 370H-45 motor, featuring a 45:1 gear ratio, and a TSINY-8370 Hall-effect encoder with 12 pulses per rotation (PPR). The encoder provides feedback on the motor shaft's position and direction, enabling real-time velocity calculations.

- **Motor**: TS-25GA 370H-45 (45:1 gear ratio)
- **Encoder**: TSINY-8370 (12 PPR)

The encoder produces 24 pulses per revolution (PPR) due to the dual-channel setup, meaning that each complete rotation of the output shaft generates 24 pulses. This allows for precise monitoring of motor position and velocity.

### Wiring & Connections  

- The motor has 6 wires—two for the motor itself and three for the encoder.
- The motor wires connect to an L293D H-Bridge motor driver, while the encoder’s channels connect to digital pins 2 and 3 on the microcontroller.
- An external power source provides 12V for the motor and 5V for the encoder.

### Controller Setup  
The microcontroller, in this case, an Arduino, manages the PWM signal sent to the motor driver. The PWM duty cycle adjusts the motor speed, while the encoder provides feedback for velocity measurement. The main challenge is calculating the velocity and using the PID algorithm to adjust the motor speed accordingly.

## What is a PID Controller?  
A **PID** (Proportional-Integral-Derivative) controller is a feedback loop mechanism designed to maintain a desired output by adjusting the input based on three error components:

- **Proportional (P)**: Adjusts the output in proportion to the current error.
- **Integral (I)**: Accounts for past errors, helping to eliminate steady-state error.
- **Derivative (D)**: Predicts future error based on its rate of change.

In this project, the PID controller continuously adjusts the motor speed by modifying the PWM signal based on the error between the desired and actual velocity, ensuring stable and accurate motor control.

## Programming the Controller  

The controller is programmed using Arduino, with the main logic centered around reading encoder data, calculating velocity, and applying the PID algorithm to adjust the motor's speed.

### PID Control Function  
The core of the PID control is the `computePID` function:

```cpp
double computePID(double Input) {  
  currentTime = millis();  // Time in milliseconds
  elapsedTime = (double)(currentTime - previousTime);
  
  error = Setpoint - Input;  
  cumError += error * elapsedTime;  
  rateError = (error - lastError) / elapsedTime;
  
  double output = kp*error + ki*cumError + kd*rateError;
  
  lastError = error;  
  previousTime = currentTime;  
  
  return constrain(map(output, 0, 200, 0, 255), 0, 255);
}
