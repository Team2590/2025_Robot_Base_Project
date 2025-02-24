package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.SafetyChecker;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private NeutralModeValue neutralMode;
  private double rotationCount;
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
  }

  @Override
  public void updateInputs(ElevatorIOInputs io) {
    // do nothing.  we have already set rotationCount when setter called
    // Log current position and target position
    io.rotationCount = this.rotationCount;
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double position) {

    System.out.println("Setting elevator position: " + position);
    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, position)) {
      this.rotationCount = position;
    } else {
      System.out.println("CAN'T MOVE ELEVATOR (SIM), safety check failed.");
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
}
