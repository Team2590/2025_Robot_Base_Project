package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SafetyChecker;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim;
  private double drumRadiusMeters;
  private double gearing;
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/cruiseVelocity", 10);
  private NeutralModeValue neutralMode;
  private double armabspos;
  private DCMotor gearBox;
  private double targetPositionDeg = 0;
  private double appliedVolts = 0.0;
  private PIDController positionController = new PIDController(0.1, 0, 0);
  private double maxVoltage = 12.0;
  private double simulatedElevatorPosition = 0;

  public ArmIOSim(
      DCMotor gearbox,
      double gearing,
      double armMassKg,
      double armLengthMeters,
      boolean simulateGravity) {
    this.gearBox = gearbox;
    this.gearing = gearing;

    // Calculate moment of inertia for a point mass at the end of the arm
    double momentOfInertia = armMassKg * Math.pow(armLengthMeters, 2);

    // Configure for 0 to 144 degrees with 0 at 7 o'clock
    // 0 degrees = 210 degrees (7 o'clock)
    // 144 degrees = 354 degrees (2 o'clock)
    double minAngle = Units.degreesToRadians(210); // 7 o'clock
    double maxAngle = Units.degreesToRadians(354); // 2 o'clock

    armSim =
        new SingleJointedArmSim(
            gearbox,
            gearing,
            momentOfInertia,
            armLengthMeters,
            minAngle,
            maxAngle,
            false,
            minAngle // Start at 7 o'clock
            );

    // Configure PID controller
    positionController.setTolerance(Units.degreesToRadians(1));
  }

  @Override
  public void updateInputs(ArmIOInputs io) {
    // Convert current position back to 0-144 degree range
    double currentPositionRad = armSim.getAngleRads();
    double currentPositionDeg = Units.radiansToDegrees(currentPositionRad) - 210;
    if (currentPositionDeg < 0) currentPositionDeg += 360;

    // Calculate control voltage based on position error
    double targetPositionRad = Units.degreesToRadians(210 + targetPositionDeg);
    double controlVoltage = positionController.calculate(currentPositionRad, targetPositionRad);

    // Limit the voltage to motor capabilities
    controlVoltage = Math.min(Math.max(controlVoltage, -maxVoltage), maxVoltage);

    // Apply voltage to the simulation
    armSim.setInputVoltage(controlVoltage);
    armSim.update(Constants.loopPeriodSecs);

    // Update inputs with current state
    io.armabspos = currentPositionDeg; // Report position in 0-144 degree range
    io.connected = true;
    io.positionRads = Units.degreesToRadians(currentPositionDeg);
    io.velocityRadsPerSec = armSim.getVelocityRadPerSec();
    io.appliedVoltage = controlVoltage;
    io.supplyCurrentAmps = armSim.getCurrentDrawAmps();
    io.tempCelsius = 30 + Math.abs(armSim.getCurrentDrawAmps()) * 0.1;
  }

  @Override
  public void updateTunableNumbers() {}

  public void setSimulatedElevatorPosition(double position) {
    this.simulatedElevatorPosition = position;
  }

  @Override
  public void setPosition(double positionDeg) {
    // Constrain position to 0-144 degrees
    double constrainedPosition = Math.min(Math.max(positionDeg, 0), 144);

    System.out.println("Setting Arm Position: " + constrainedPosition + " degrees");
    if (SafetyChecker.isSafe(
        SafetyChecker.MechanismType.ARM_MOVEMENT, simulatedElevatorPosition, constrainedPosition)) {
      // Convert to radians with offset (0 degrees = 210 degrees)
      double targetRad = Units.degreesToRadians(210 + constrainedPosition);
      armSim.setState(targetRad, 0); // Set position and zero velocity
      targetPositionDeg = constrainedPosition;
    } else {
      System.out.println("CAN'T MOVE ARM (SIM), elevator not in valid position.");
    }
  }

  @Override
  public void stop() {
    if (neutralMode == NeutralModeValue.Brake) {
      armSim.setInputVoltage(0);
      armSim.setState(armSim.getAngleRads(), 0);
    } else {
      armSim.setInputVoltage(0);
    }
  }

  @Override
  public void setPower(DutyCycleOut power) {
    // Convert duty cycle to voltage
    appliedVolts = power.Output * maxVoltage;
  }
}
