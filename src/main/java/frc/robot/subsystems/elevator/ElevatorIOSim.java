package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;

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
  }

  public void updateTunableNumbers() {}

  public void setPosition(double position) {

  }

  public void stop() {

  }

  public void resetRotationCount() {

  }

  public void setNeutralMode(NeutralModeValue mode) {

  }
}
