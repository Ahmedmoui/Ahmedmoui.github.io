---
layout: post  
title: PID Motor Velocity Controller  
description: A hands-on project aimed at designing and implementing a PID motor velocity controller for precise motor speed regulation using real-time feedback.  
skills:  
  - Control Algorithms  
  - Motor Control  
  - Embedded Systems  
  - Arduino Programming  
  - Feedback Systems  
  - Systems Integration  

main-image: /PID motor controler diagram.png 
---

# PID Motor Velocity Controller: A Practical Approach to Precision Speed Control  

## Project Overview  
The challenge of this project was to design and implement a system capable of controlling the velocity of a motor in real-time, using a PID controller. Precision control of motor speed is a critical requirement in various applications like robotics and automation, where fine adjustments are necessary for optimal performance. The goal was to develop a robust system that could adjust motor speed dynamically based on feedback from an encoder, ensuring the motor's velocity matched a specified setpoint.

## Problem Description  
Many applications, including robotic arms, drones, and automated machines, rely on motors that need to operate at consistent speeds. A common challenge is maintaining a stable motor velocity despite external disturbances or varying load conditions. A simple open-loop control system, such as direct PWM control, might not suffice, as it does not adjust for errors in motor speed or external forces. To solve this, we implemented a closed-loop feedback control system using a **PID controller**.

The system would need to:
- Measure the motor's velocity in real-time using an encoder.
- Compare the measured velocity with the desired setpoint.
- Adjust the motor speed dynamically by adjusting the PWM duty cycle, based on feedback.

## What I Did  
I designed and implemented a closed-loop motor velocity controller using a PID algorithm. The system comprised the following components:

### Hardware Components  
- **TS-25GA 370H-45 Motor**: This motor features a 45:1 gear ratio, providing mechanical advantage and precise speed control.
- **TSINY-8370 Encoder**: A Hall-effect encoder that outputs 12 pulses per revolution (PPR), providing position feedback for velocity calculation.
- **L293D H-Bridge Motor Driver**: Used to control the motor's speed and direction via PWM.
- **Arduino**: The microcontroller used to implement the PID control algorithm, read encoder data, and adjust the motor speed.

The encoder produces pulses that represent angular displacement. By counting these pulses and measuring the time between them, the motor's velocity can be calculated in degrees per second. This velocity is fed into the PID controller to adjust the PWM signal sent to the motor, maintaining a steady speed.

### Programming the PID Controller  
The core of the system lies in the PID control algorithm, which adjusts the motor speed based on the error between the desired and actual velocity. The algorithm calculates three terms:

1. **Proportional (P)**: Reacts to the current error.
2. **Integral (I)**: Accounts for accumulated past errors.
3. **Derivative (D)**: Predicts future error based on the rate of change.

Using an Arduino, I implemented the PID controller, where the proportional, integral, and derivative constants (`kp`, `ki`, `kd`) were adjusted to optimize motor speed control.

#### Key Functions Implemented  
- **`computePID` Function**: Calculates the control output based on the PID algorithm.
- **`AngularVelocity` Function**: Computes the motor's velocity by measuring time intervals between encoder pulses.
- **Main Loop**: Reads encoder data, computes the error, and updates the motor's speed using the PID controller.

```cpp
// PID Motor Velocity Controller Arduino Code

#define enable 11
#define in1 10
#define in2 9

double TargetSpeed = 500;  // Target speed (degrees per second)
double kp = 0.005, ki = 0.0005, kd = 0.00;  // PID constants
long previousTime = 0;
long currentTime;
double elapsedTime;
double error, lastError = 0, cumError = 0, rateError;
double Setpoint, Input, Output;
volatile long counter = 0;  // Encoder pulse counter

void setup() {
  Serial.begin(9600);
  pinMode(enable, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(2, INPUT_PULLUP);  // Encoder channel A
  pinMode(3, INPUT_PULLUP);  // Encoder channel B
  attachInterrupt(0, ai0, RISING);  // Interrupt for encoder pulse on channel A
  attachInterrupt(1, ai1, RISING);  // Interrupt for encoder pulse on channel B
}

void loop() {
  Setpoint = TargetSpeed;  // Desired motor speed
  long pos = counter / 3;  // Calculate motor position
  Input = AngularVelocity(pos);  // Calculate current motor speed
  
  if (isnan(Input) == 0) {
    Output = computePID(Input);  // Apply PID control
  }
  motor_speed(Output);  // Set motor speed based on PID output
}

// Function to compute PID output
double computePID(double Input) {  
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  
  error = Setpoint - Input;  
  cumError += error * elapsedTime;  
  rateError = (error - lastError) / elapsedTime;
  
  double output = kp*error + ki*cumError + kd*rateError;
  
  lastError = error;  
  previousTime = currentTime;  
  
  return constrain(map(output, 0, 200, 0, 255), 0, 255);
}

// Function to calculate angular velocity from encoder pulses
double AngularVelocity(long pos) {
  return pos / 3.0;  // Motor speed in degrees per second
}

// Function to control motor speed via PWM
void motor_speed(double speed) {
  analogWrite(enable, speed);  // Adjust motor speed based on PID output
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// Interrupt service routines for encoder pulses
void ai0() {
  counter++;
}

void ai1() {
  counter--;
}
```