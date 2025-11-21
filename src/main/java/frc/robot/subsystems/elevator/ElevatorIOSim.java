package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SafetyChecker;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private double drumCircumfrenceMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 10);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("Arm/acceleration", 15);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("Arm/jerk", 20);
  private NeutralModeValue neutralMode;
  private double rotationCount;
  private DCMotor gearBox;
  private double requestedPositionMeters = 0;
  private boolean holding = true;

  public ElevatorIOSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumCircumfrenceMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      double startingHeightMeters,
      double... measurementStdDevs) {
    this.gearBox = gearbox;
    this.drumCircumfrenceMeters = drumCircumfrenceMeters;
    this.gearing = gearing;

    elevatorSim =
        new ElevatorSim(
            gearbox,
            gearing,
            carriageMassKg,
            drumCircumfrenceMeters / (2 * Math.PI),
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeightMeters,
            measurementStdDevs);
  }

  @Override
  public void updateInputs(ElevatorIOInputs io) {
    elevatorSim.update(Constants.loopPeriodSecs);
    rotationCount = positionToRotations(elevatorSim.getPositionMeters());
    io.rotationCount = rotationCount;
    io.connected = true;
    io.positionRads = Units.rotationsToRadians(rotationCount);
    io.velocityRadsPerSec =
        Units.rotationsToRadians(positionToRotations(elevatorSim.getVelocityMetersPerSecond()));
    io.appliedVoltage = elevatorSim.getCurrentDrawAmps();
    io.supplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    io.torqueCurrentAmps = elevatorSim.getCurrentDrawAmps();
    io.tempCelsius = 30;
    if (holding) elevatorSim.setState(requestedPositionMeters, cruiseVelocity.get());
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double position) {
    double elevatorPos = this.rotationCount;

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, elevatorPos)) {
      double positionMeters = position * drumCircumfrenceMeters / gearing;
      requestedPositionMeters = positionMeters;
      elevatorSim.setState(positionMeters, cruiseVelocity.get());
    } else {
      // System.out.println("CAN'T MOVE ELEVATOR (SIM), safety check failed.");
    }
  }

  @Override
  public double getTargetPosition() {
    return requestedPositionMeters;
  }

  @Override
  public void stop() {
    double currentPositionMeters = elevatorSim.getPositionMeters();

    if (neutralMode == NeutralModeValue.Brake) {
      elevatorSim.setState(currentPositionMeters, 0.0);
    } else {
      holding = false;
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

  private double positionToRotations(double positionMeters) {
    return positionMeters / (drumCircumfrenceMeters * gearing);
  }
}
