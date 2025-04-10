# Robot Controls Documentation

## Overview
This document outlines the control mappings for both the driver (using joysticks) and operator (using the controller app) of the robot. The controls are organized by functionality and user role.

## Driver Controls (Joysticks)

### Real Robot Controls

#### Left Joystick
- **Default**: Drive control (forward/backward, strafe)
- **Button 2**: Stow mechanism
- **Button 3**: Score at Level 1
- **Button 4**: Set arm to coral scoring position
- **Button 8**: Manual climb control
- **POV Right**: Score at Level 2
- **POV Down**: Score at Level 3
- **POV Left**: Score at Level 4
- **POV Up**: Score algae barge
- **Trigger**: End effector outtake (when right joystick button 4 is not pressed)

#### Right Joystick
- **Button 2**: Drive and auto score command
- **Button 3**: Intake coral (no handoff)
- **Button 4**: Score processor (when combined with left joystick trigger)
- **Button 5**: Reset robot pose
- **Button 8**: Reset elevator rotation count
- **Button 11**: Deploy mechanism
- **Button 12**: Prepare for climb
- **Button 13**: Hold algae voltage
- **Button 14**: Manual elevator down
- **Button 15**: Manual elevator up
- **Button 16**: Climb
- **POV Right**: Grab algae at Level 3
- **POV Left**: Grab algae at Level 2
- **POV Up**: End effector intake voltage
- **POV Down**: Intake coral outtake (spit)
- **Trigger**: 
  - With button 4: Intake algae from ground
  - Without button 4: Intake coral from ground with handoff

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

## Operator Controls (Controller App)

The operator uses a JavaFX-based controller application with the following features:

### Main Control Grid
- **Left-L4**: Select left side Level 4 scoring position
- **Right-L4**: Select right side Level 4 scoring position
- **Left-L3**: Select left side Level 3 scoring position
- **Right-L3**: Select right side Level 3 scoring position
- **Left-L2**: Select left side Level 2 scoring position
- **Right-L2**: Select right side Level 2 scoring position

### Bottom Controls
- **Connect**: Connect to robot
- **Refresh**: Refresh the interface

### Visual Feedback
- Selected buttons are highlighted in green (#00CC66)
- Unselected buttons are blue (#0080FF)

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
