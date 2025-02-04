package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ArmIOSim implements ArmIO {
  private ElevatorSim elevatorSim;
  private double drumRadiusMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 10);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("Arm/acceleration", 15);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("Arm/jerk", 20);
  private NeutralModeValue neutralMode;
  private double armabspos;
  private DCMotor gearBox;
  private double requestedPositionMeters = 0;
  private boolean holding = true;

  public ArmIOSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      double startingHeightMeters) {
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
            startingHeightMeters);
  }

  @Override
  public void updateInputs(ArmIOInputs io) {
    elevatorSim.update(Constants.loopPeriodSecs);
    armabspos = Math.cos(positionToRotations(elevatorSim.getPositionMeters()));

    io.armabspos = armabspos;
    io.connected = true;
    io.velDegreesPerSecond =
        Units.rotationsToDegrees(velocityToRotations(elevatorSim.getVelocityMetersPerSecond()));
    io.appliedVoltage = elevatorSim.getCurrentDrawAmps();
    io.currentAmps = elevatorSim.getCurrentDrawAmps();
    // io.torqueCurrentAmps = elevatorSim.getCurrentDrawAmps();
    // io.tempCelsius = 30;
    if (holding) elevatorSim.setState(requestedPositionMeters, cruiseVelocity.get());
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double position) {
    double positionMeters = position * 2 * Math.PI * drumRadiusMeters / gearing;

    requestedPositionMeters = positionMeters;

    elevatorSim.setState(positionMeters, 0);
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
  public void setPower(DutyCycleOut power) {

    elevatorSim.setInputVoltage(power.Output);
  }

  private double positionToRotations(double positionMeters) {
    return positionMeters / (2 * Math.PI * drumRadiusMeters / gearing);
  }

  private double velocityToRotations(double velocityMeters) {
    return velocityMeters / (drumRadiusMeters / gearing);
  }
}
