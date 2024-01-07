# WAVE 2023-2024 swerve bot

This repository contains our 2023-2024 off-season swerve bot code.  
The robot uses the MK4 swerve modules, Pigeon 2.0 gyro, and Neo motors.  
The autonomous command follows Apriltags recognized by a Limelight 3.0 on the host "limelight".  
The teleop command uses the following controls:
- Left joystick: move robot
- Right joystick left/right: turn robot
- X button: change the controls to robot-relative rather than field-relative
- Start button: zero the gyro (reorient robot to understand current angle as "forward")