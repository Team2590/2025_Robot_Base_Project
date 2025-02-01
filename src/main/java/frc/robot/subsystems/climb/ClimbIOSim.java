package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbIOSim implements ClimbIO {
  private DCMotorSim climbSim;
  private DCMotor gearbox;
  private double rotationCount;
  private double jkgMetersSquared;
  private double gearing;
  private boolean holding = true;
  private double requestedPositionMeters = 0;
  private NeutralModeValue neutralMode;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 10);

  public ClimbIOSim(
      DCMotor gearbox, double jkgMetersSquared, double gearing, double... measurmentStdDevs) {
    this.gearbox = gearbox;
    this.gearing = gearing;
    this.jkgMetersSquared = jkgMetersSquared;
    climbSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, jkgMetersSquared, gearing),
            gearbox,
            measurmentStdDevs);
  }

  @Override
  public void updateInputs(ClimbIOInputs io) {
    climbSim.update(Constants.loopPeriodSecs);
    io.leaderRotationCount = rotationCount;
    io.connected = true;
    io.leaderPositionRads = Units.rotationsToRadians(rotationCount);
    io.velocityRadsPerSec = climbSim.getAngularVelocityRadPerSec();
    io.appliedVoltage = climbSim.getCurrentDrawAmps();
    io.supplyCurrentAmps = climbSim.getCurrentDrawAmps();
    io.torqueCurrentAmps = climbSim.getCurrentDrawAmps();
    io.tempCelsius = 30;
    if (holding) climbSim.setState(requestedPositionMeters, cruiseVelocity.get());
  }

  @Override
  public void run(double voltage) {
    climbSim.setInputVoltage(voltage);
    climbSim.setState(requestedPositionMeters, 14);
  }

  @Override
  public void setVoltage(double voltage) {
    climbSim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    climbSim.setState(requestedPositionMeters, 0);
  }

  @Override
  public void resetRotationCount() {
    rotationCount = 0;
  }

  @Override
  public double getRotationCount() {
    return rotationCount;
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    neutralMode = mode;
  }
}
