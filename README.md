# MobileRobot-esp32s3

This code runs a companion controller -- the `esp32s3` inside the `Atom S3 Lite`for a Raspberry Pi 4. This controller accepts body velocity commands and converts it into two PWM signals assuming a differential drive platform. The motor commands are uncalibrated and open loop. 

## Logs
A compact and more readable version of the code in `wheeledrobot`
As of 6/20/24 still accepts wheel speed commands
Need to bring over the code for conversion from robot velocities to wheel speeds
As of 07/01/24 uses the code in `wheeledrobot` to convert v,omega into wheel speeds. no deadzone
During Fall 2025, reintroduce deadzone and also a watchdog timer to handle infrequent commands from main computer.

