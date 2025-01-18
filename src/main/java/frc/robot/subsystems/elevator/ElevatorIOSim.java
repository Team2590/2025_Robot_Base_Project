package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private double drumRadiusMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 0);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("Arm/acceleration", 0);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("Arm/jerk", 0);
  private NeutralModeValue neutralMode;
  private double rotationCount;
  private DCMotor gearBox;
  
  public ElevatorIOSim(
    DCMotor gearbox, 
    double gearing, 
    double carriageMassKg,
    double drumRadiusMeters,
    double minHeightMeters,
    double maxHeightMeters, 
    boolean simulateGravity, 
    double startingHeightMeters, 
    double... measurementStdDevs
  ) {
    this.gearBox = gearbox;
    this.drumRadiusMeters = drumRadiusMeters;
    this.gearing = gearing;  

    elevatorSim = new ElevatorSim(
      gearbox, 
      gearing, 
      carriageMassKg,
      drumRadiusMeters,
      minHeightMeters,
      maxHeightMeters, 
      simulateGravity, 
      startingHeightMeters, 
      measurementStdDevs
    );
  }

  public void updateInputs(ElevatorIOInputs io) {
    elevatorSim.update(Constants.loopPeriodSecs);
    rotationCount = positionToRotations(elevatorSim.getPositionMeters());
    io.rotationCount = rotationCount;
    io.connected = true;
    io.positionRads = Units.rotationsToRadians(rotationCount);
    io.velocityRadsPerSec = Units.rotationsToRadians(positionToRotations(elevatorSim.getVelocityMetersPerSecond()));
    io.appliedVoltage = elevatorSim.getCurrentDrawAmps();
    io.supplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    io.torqueCurrentAmps = elevatorSim.getCurrentDrawAmps();
    io.tempCelsius = 30;
  }

  public void updateTunableNumbers() {}

  public void setPosition(double position) {
    double positionMeters = position * 2 * Math.PI * drumRadiusMeters / gearing;

    elevatorSim.setState(positionMeters, cruiseVelocity.get());
  }

  public void stop() {
    double currentPositionMeters = elevatorSim.getPositionMeters();

    if (neutralMode == NeutralModeValue.Brake) {
      elevatorSim.setState(currentPositionMeters, 0.0);
    }
  }

  public void resetRotationCount() {
    rotationCount = 0;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    neutralMode = mode;
  }

  private double positionToRotations(double positionMeters) {
    return positionMeters / (2 * Math.PI * drumRadiusMeters / gearing);
  }
}
