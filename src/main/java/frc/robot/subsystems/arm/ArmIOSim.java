package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim;
  private double drumRadiusMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 10);
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
  }

  @Override
  public void updateInputs(ArmIOInputs io) {
    armSim.update(Constants.loopPeriodSecs);
    io.armabspos = armSim.getAngleRads();
    io.connected = true;
    io.appliedVoltage = armSim.getCurrentDrawAmps();
    io.positionRads = Units.rotationsToRadians(armSim.getAngleRads());
    io.velocityRadsPerSec = armSim.getVelocityRadPerSec();
    io.supplyCurrentAmps = armSim.getCurrentDrawAmps();
    io.tempCelsius = 30;
    if (holding) armSim.setState(requestedPositionMeters, cruiseVelocity.get());
  }

  @Override
  public void updateTunableNumbers() {}

  @Override
  public void setPosition(double position) {
    double positionMeters = position * 2 * Math.PI * drumRadiusMeters / gearing;

    requestedPositionMeters = positionMeters;

    armSim.setState(positionMeters, 0);
  }

  public double getSetpoint() {
    return requestedPositionMeters;
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

    armSim.setInputVoltage(power.Output);
  }
}
