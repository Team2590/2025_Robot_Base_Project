# Robot Controls Documentation

## Overview
This document outlines the control mappings for both the driver (using joysticks) and operator (using an Xbox controller) of the robot. The controls are organized by functionality and user role.

## Driver Controls (Joysticks)

### Real Robot Controls

#### Left Joystick
- **Default**: Drive control (forward/backward, strafe)
- **Button 2**: Stow mechanism
- **Button 8**: Manual climb control
- **POV Right**: Score at Level 2
- **POV Down**: Score at Level 3
- **POV Left**: Score at Level 4
- **POV Up**: Score algae barge
- **Trigger**:
  - With Right Joystick Button 4: Score in processor
  - If target is Level 1: Outtake coral
  - If target is not Level 1: Arm follow through

#### Right Joystick
- **Button 2**: Drive and auto score command
- **Button 3**: Grab algae from reef
- **Button 4**: (Used in combination with other buttons)
- **Button 5**: Reset drive pose
- **Button 8**: Reset elevator rotation count
- **Button 11**: Deploy climb mechanism
- **Button 12**: Prepare for climb
- **Button 13**: Manual elevator down
- **Button 14**: Manual elevator up
- **Button 16**: Climb
- **POV Up**: End effector intake
- **POV Down**: Intake coral outtake (spit)
- **Trigger**:
  - With button 4: Intake algae from ground
  - Without button 4 (target is L1): Intake coral from ground to L1
  - Without button 4 (target is not L1): Intake coral from ground with handoff

### Simulation Mode Controls

#### Left Joystick
- **Button 1**: Drive to source intake
- **Button 2**: Drive to target
- **Button 4**: Move elevator to home position
- **Button 5**: Move elevator to maximum position
- **Button 6**: Move arm to minimum position
- **Button 7**: Move arm to maximum position
- **Button 8**: Score at Level 3
- **Button 9**: Score processor

## Operator Controls (Xbox Controller)

The operator uses an Xbox controller for target selection.

- **X Button**: Set target to Level 4
- **B Button**: Set target to Level 2
- **A Button**: Set target to Level 3
- **Y Button**: Set target to Level 1
- **Left Bumper**: Set target side to Left
- **Right Bumper**: Set target side to Right
- **Right Trigger**: Run end effector for algae intake

## Additional Notes

### Mode Selection
The robot operates in different modes:
- **Real Robot Mode**: Uses the full set of controls for actual robot operation
- **Simulation Mode**: Uses a simplified set of controls for testing and development
- The mode is automatically selected based on whether the code is running on a real robot or in simulation

### Safety Features
- The controller app includes connection status monitoring
- The app automatically disconnects when closed
- Network table communication is used for robot control

### Scoring Levels
The robot supports scoring at multiple levels:
- Level 1 (L1)
- Level 2 (L2)
- Level 3 (L3)
- Level 4 (L4)
- Algae Barge
- Processor

### Game Piece Handling
The robot can handle different types of game pieces:
- Coral (with and without handoff)
- Algae
- Support for both intake and outtake operations
