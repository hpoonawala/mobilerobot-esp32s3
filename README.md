# MobileRobot-esp32s3

This code runs a companion controller -- the `esp32s3` inside the `Atom S3 Lite`for a Raspberry Pi 4. This controller accepts body velocity commands and converts it into two PWM signals assuming a differential drive platform. The motor commands are uncalibrated and open loop. 

## Structure 

The main routine is in  `main/pimotorcontrol.c`. It sets up a task for handling PWM, a watchdog timer to stop motors after a timeout from last command, and then the receiver task. The receiver task will repeatedly try to read from USB, and if it gets a structured message (see below) it will parse it to extract the desired velocities and convert them into PWM commands for the right and left motor. 

### Wheel Speed Conversion

The forward speed $v$ and angular velocity $\omega$ are read from the message and converted into wheel speed duty cycles. 
The left and right PWM duty cycles are simply $v-\omega$ and $v+\omega$, where the $255$ is $100$\%.  
Due to saturation, the values of $v$ and $\omega$ are modified to prioritize angular velocity. 
Large angular velocities are achieved at the expense of forward velocity.

Saturation and deadzone effects make this open-loop control method highly nonlinear and therefore difficult to tune. Velocity-level controllers with small gains cause the robot to stay in place, but increasing the gains cause large turning rates in-place that confuse LiDAR-only SLAM. 

The project intended to demonstrate performance of a simple nonlinear end-to-end LiDAR-based control for navigation with guarantees that work for a broad range of models. The controller works well despite the simple wheel speed conversion that doesn't eliminate non-linearities. 

### Structured Message 

This structure is mostly due to legacy code and could use an overhaul. 
- `S.0` : stop wheels (PWM with duty cycle $0$)
- `D.+VVV.sWWW.0` 
    - `D`: differential drive command 
    - `VVV`: three digit integer $000 <=$ VVV $< 255$ for forward velocity 
    - `s`: sign `+` or `-`
    - `WWW`: three digit integer $000 <=$ WWW $< 255$  for magnitude of angular velocity
    - `0`: unused digit for watchdog timeout setting
- `C.RRR.GGG.BBB`: flash LED with color defined by three-digit RGB values (eg. $068$ not $68$). 

## Logs

A compact and more readable version of the code in `wheeledrobot`
As of 6/20/24 still accepts wheel speed commands
Need to bring over the code for conversion from robot velocities to wheel =
As of 07/01/24 uses the code in `wheeledrobot` to convert v,omega into wheel speeds. no deadzone
During Fall 2025, reintroduce deadzone and also a watchdog timer to handle infrequent commands from main computer.

