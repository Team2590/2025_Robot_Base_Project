package frc.robot.subsystems.peashooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.PeaShooterConstants;

public class PeaShooterIOSim implements PeaShooterIO {
  private final DCMotorSim leftSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.004, PeaShooterConstants.FLYWHEEL_REDUCTION),
          DCMotor.getFalcon500(1),
          PeaShooterConstants.FLYWHEEL_REDUCTION);
  private final DCMotorSim rightSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.004, PeaShooterConstants.FLYWHEEL_REDUCTION),
          DCMotor.getFalcon500(1),
          PeaShooterConstants.FLYWHEEL_REDUCTION);
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          PeaShooterConstants.HOOD_REDUCTION,
          SingleJointedArmSim.estimateMOI(0.2, 1.0), // Length 0.2m, Mass 1kg guess
          0.2, // Length
          Units.degreesToRadians(PeaShooterConstants.HOOD_MIN_POS),
          Units.degreesToRadians(PeaShooterConstants.HOOD_MAX_POS),
          true, // Simulate gravity
          Units.degreesToRadians(PeaShooterConstants.HOOD_MIN_POS));

  private final PIDController leftController =
      new PIDController(
          PeaShooterConstants.FLYWHEEL_kP,
          PeaShooterConstants.FLYWHEEL_kI,
          PeaShooterConstants.FLYWHEEL_kD);
  private final PIDController rightController =
      new PIDController(
          PeaShooterConstants.FLYWHEEL_kP,
          PeaShooterConstants.FLYWHEEL_kI,
          PeaShooterConstants.FLYWHEEL_kD);
  private final PIDController hoodController =
      new PIDController(
          PeaShooterConstants.HOOD_kP, PeaShooterConstants.HOOD_kI, PeaShooterConstants.HOOD_kD);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;

  private boolean leftClosedLoop = false;
  private boolean rightClosedLoop = false;
  private boolean hoodClosedLoop = false;

  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;
  private double hoodSetpoint = 0.0;

  @Override
  public void updateInputs(PeaShooterIOInputs inputs) {
    if (leftClosedLoop) {
      leftAppliedVolts =
          leftController.calculate(leftSim.getAngularVelocityRadPerSec(), leftSetpoint)
              + PeaShooterConstants.FLYWHEEL_kV * leftSetpoint;
    }
    if (rightClosedLoop) {
      rightAppliedVolts =
          rightController.calculate(rightSim.getAngularVelocityRadPerSec(), rightSetpoint)
              + PeaShooterConstants.FLYWHEEL_kV * rightSetpoint;
    }
    if (hoodClosedLoop) {
      hoodAppliedVolts = hoodController.calculate(hoodSim.getAngleRads(), hoodSetpoint);
    }

    leftAppliedVolts = MathUtil.clamp(leftAppliedVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightAppliedVolts, -12.0, 12.0);
    hoodAppliedVolts = MathUtil.clamp(hoodAppliedVolts, -12.0, 12.0);

    leftSim.setInputVoltage(leftAppliedVolts);
    rightSim.setInputVoltage(rightAppliedVolts);
    hoodSim.setInputVoltage(hoodAppliedVolts);

    leftSim.update(Constants.loopPeriodSecs);
    rightSim.update(Constants.loopPeriodSecs);
    hoodSim.update(Constants.loopPeriodSecs);

    inputs.leftFlywheelConnected = true;
    inputs.leftFlywheelVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
    inputs.leftFlywheelAppliedVolts = leftAppliedVolts;
    inputs.leftFlywheelCurrentAmps = leftSim.getCurrentDrawAmps();

    inputs.rightFlywheelConnected = true;
    inputs.rightFlywheelVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
    inputs.rightFlywheelAppliedVolts = rightAppliedVolts;
    inputs.rightFlywheelCurrentAmps = rightSim.getCurrentDrawAmps();

    inputs.hoodConnected = true;
    inputs.hoodPositionRad = hoodSim.getAngleRads();
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftClosedLoop = false;
    leftAppliedVolts = volts;
  }

  @Override
  public void setRightVoltage(double volts) {
    rightClosedLoop = false;
    rightAppliedVolts = volts;
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodClosedLoop = false;
    hoodAppliedVolts = volts;
  }

  @Override
  public void setLeftVelocity(double velocityRadPerSec) {
    leftClosedLoop = true;
    leftSetpoint = velocityRadPerSec;
  }

  @Override
  public void setRightVelocity(double velocityRadPerSec) {
    rightClosedLoop = true;
    rightSetpoint = velocityRadPerSec;
  }

  @Override
  public void setHoodPosition(double positionRad) {
    hoodClosedLoop = true;
    hoodSetpoint = positionRad;
  }

  @Override
  public void stop() {
    setLeftVoltage(0.0);
    setRightVoltage(0.0);
    setHoodVoltage(0.0);
  }
}
