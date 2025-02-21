package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotContainer;
import frc.robot.util.SafetyChecker;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private double drumRadiusMeters;
  private double gearing;
  private NeutralModeValue neutralMode;
  private double rotationCount;
  private double appliedVolts = 0.0;
  private PIDController positionController = new PIDController(0.5, 0, 0);
  private double actualPositionSimulated = 0;

  public ElevatorIOSim(
      double gearing, double carriageMassKg, double drumRadiusMeters, boolean simulateGravity) {
    this.gearing = gearing;
    this.drumRadiusMeters = drumRadiusMeters;

    positionController.setTolerance(0.01); // 1% tolerance
  }

  @Override
  public void updateInputs(ElevatorIOInputs io) {
    // Calculate control voltage based on position error
    io.rotationCount = actualPositionSimulated;
    io.connected = true;
    io.positionRads = Units.rotationsToRadians(io.rotationCount);
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double positionRotations) {

    double armPos = RobotContainer.getArm().getAbsolutePosition();

    if (SafetyChecker.isSafe(
        SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, positionRotations, armPos)) {
      this.actualPositionSimulated = positionRotations;
    } else {
      System.out.println("CAN'T MOVE ELEVATOR (SIM), arm not in valid position.");
    }
  }

  @Override
  public double getTargetPosition() {
    return this.actualPositionSimulated;
  }

  @Override
  public void stop() {
    if (neutralMode == NeutralModeValue.Brake) {
      elevatorSim.setInputVoltage(0);
      elevatorSim.setState(elevatorSim.getPositionMeters(), 0);
    } else {
      elevatorSim.setInputVoltage(0);
    }
  }

  @Override
  public void resetRotationCount() {
    rotationCount = 0;
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    neutralMode = mode;
  }

  @Override
  public void setVoltage(VoltageOut volts) {
    double armPos = RobotContainer.getArm().getAbsolutePosition();
    double elevatorPos = this.rotationCount;

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, elevatorPos, armPos)) {
      appliedVolts = volts.Output;
      elevatorSim.setInputVoltage(appliedVolts);
    } else {
      System.out.println("CAN'T MOVE ELEVATOR (SIM), arm not in valid position");
      appliedVolts = 0.0;
      elevatorSim.setInputVoltage(0.0);
    }
  }
}
