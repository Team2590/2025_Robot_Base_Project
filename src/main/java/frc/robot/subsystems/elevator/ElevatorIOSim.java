package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SafetyChecker;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private double drumRadiusMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("Elevator/cruiseVelocity", 2.0);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/acceleration", 1.0);
  private NeutralModeValue neutralMode;
  private double rotationCount;
  private DCMotor gearBox;
  private double targetPositionMeters = 0;
  private double appliedVolts = 0.0;
  private PIDController positionController = new PIDController(0.5, 0, 0);
  private double maxVoltage = 12.0;
  private double requestedPositionMeters = 0;

  public ElevatorIOSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      double startingHeightMeters,
      double... measurementStdDevs) {
    this.gearBox = gearbox;
    this.drumRadiusMeters = drumRadiusMeters;
    this.gearing = gearing;

    elevatorSim =
        new ElevatorSim(
            gearbox,
            gearing,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeightMeters,
            measurementStdDevs);

    positionController.setTolerance(0.01); // 1 cm tolerance
  }

  @Override
  public void updateInputs(ElevatorIOInputs io) {
    // Calculate control voltage based on position error
    double currentPositionMeters = elevatorSim.getPositionMeters();
    double controlVoltage =
        positionController.calculate(currentPositionMeters, targetPositionMeters);

    // Limit the voltage to motor capabilities
    controlVoltage = Math.min(Math.max(controlVoltage, -maxVoltage), maxVoltage);

    // Apply voltage to the simulation
    elevatorSim.setInputVoltage(controlVoltage);
    elevatorSim.update(Constants.loopPeriodSecs);

    // Update inputs
    rotationCount = positionToRotations(elevatorSim.getPositionMeters());
    io.rotationCount = rotationCount;
    io.connected = true;
    io.positionRads = Units.rotationsToRadians(rotationCount);
    io.velocityRadsPerSec =
        Units.rotationsToRadians(positionToRotations(elevatorSim.getVelocityMetersPerSecond()));
    io.appliedVoltage = controlVoltage;
    io.supplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    io.torqueCurrentAmps = elevatorSim.getCurrentDrawAmps();
    io.tempCelsius =
        30 + Math.abs(elevatorSim.getCurrentDrawAmps()) * 0.1; // Simulate motor heating
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double position) {
    double armPos = RobotContainer.getArm().getAbsolutePosition();
    double elevatorPos = this.rotationCount;

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, elevatorPos, armPos)) {
      double positionMeters = position * 2 * Math.PI * drumRadiusMeters / gearing;
      requestedPositionMeters = positionMeters;
      elevatorSim.setState(positionMeters, cruiseVelocity.get());
    } else {
      System.out.println("CAN'T MOVE ELEVATOR (SIM), arm not in valid position.");
    }
  }

  @Override
  public double getTargetPosition() {
    return requestedPositionMeters;
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

  private double positionToRotations(double positionMeters) {
    return positionMeters / (2 * Math.PI * drumRadiusMeters / gearing);
  }
}
