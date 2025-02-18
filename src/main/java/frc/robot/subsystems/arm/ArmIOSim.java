package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.RobotContainer;
import frc.robot.util.SafetyChecker;

public class ArmIOSim implements ArmIO {

  private double armPositionSimulated = 0;

  public ArmIOSim() {}

  @Override
  public void updateInputs(ArmIOInputs io) {
    // Convert current position to radians
    double currentPositionRad =
        armPositionSimulated; // convert armPositionSimulated to radians.  ie. 1.4 cancoder, etc.

    io.armabspos = currentPositionRad;
    io.positionRads = currentPositionRad;
  }

  @Override
  public void updateTunableNumbers() {}

  public void setSimulatedElevatorPosition(double position) {
    this.armPositionSimulated = position;
  }

  @Override
  public void setPosition(double cancoderRotations) {
    double simulatedElevatorPosition = RobotContainer.getElevator().getRotationCount();

    System.out.println("Setting Arm Position: " + cancoderRotations + " cancoder rotations");
    if (SafetyChecker.isSafe(
        SafetyChecker.MechanismType.ARM_MOVEMENT, cancoderRotations, simulatedElevatorPosition)) {

      this.armPositionSimulated = cancoderRotations;
    } else {
      System.out.println(
          "Can't move arm (sim), elevator not in valid position: " + simulatedElevatorPosition);
    }
  }

  @Override
  public void stop() {}

  @Override
  public void setPower(DutyCycleOut power) {}
}
