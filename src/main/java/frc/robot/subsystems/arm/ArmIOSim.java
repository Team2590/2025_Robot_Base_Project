package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedTunableNumber;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim;
  private double drumRadiusMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 10);
  private NeutralModeValue neutralMode;
  private double armabspos;
  private double voltage;
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

    armSim =
        new SingleJointedArmSim(
            gearbox,
            gearing,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeightMeters);
    this.armabspos = startingHeightMeters;
  }

  @Override
  public void updateInputs(ArmIOInputs io) {
    io.armpos = this.armabspos;
    io.connected = true;
    io.positionRads = armSim.getAngleRads();
    io.rotationCount = this.armabspos;
    io.velocityRadsPerSec = armSim.getVelocityRadPerSec();
    io.appliedVoltage = voltage;
    io.connected = true;
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double position) {
    this.armabspos = position;
    armSim.setState(Units.rotationsToRadians(position), 0.0);
  }

  @Override
  public void stop() {
    double currentAngleRad = armSim.getAngleRads();

    if (neutralMode == NeutralModeValue.Brake) {
      armSim.setState(currentAngleRad, 0.0);
    } else {
      holding = false;
    }
  }

  @Override
  public void setPower(DutyCycleOut power) {
    voltage = power.Output;
    armSim.setInputVoltage(power.Output);
  }
}
