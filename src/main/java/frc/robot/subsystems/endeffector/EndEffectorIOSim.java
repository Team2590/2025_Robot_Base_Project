package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0;

  public EndEffectorIOSim(DCMotor motorModel, double reduction, double inertia) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, inertia, reduction), motorModel);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }
}
