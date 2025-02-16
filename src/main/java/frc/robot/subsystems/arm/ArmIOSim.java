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
  private double targetPositionRad = 0;
  private double appliedVolts = 0.0;
  private PIDController positionController = new PIDController(0.5, 0, 0);
  private double maxVoltage = 12.0;
  private double simulatedElevatorPosition = 0;

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

    positionController.setTolerance(Units.degreesToRadians(1));
  }

  @Override
  public void updateInputs(ArmIOInputs io) {
    // Calculate control voltage based on position error
    double currentPositionRad = armSim.getAngleRads();
    double controlVoltage = positionController.calculate(currentPositionRad, targetPositionRad);

    // Limit the voltage to motor capabilities
    controlVoltage = Math.min(Math.max(controlVoltage, -maxVoltage), maxVoltage);

    // Apply voltage to the simulation
    armSim.setInputVoltage(controlVoltage);
    armSim.update(Constants.loopPeriodSecs);

    // Update inputs
    io.armabspos = armSim.getAngleRads();
    io.connected = true;
    io.appliedVoltage = controlVoltage;
    io.positionRads = armSim.getAngleRads();
    io.velocityRadsPerSec = armSim.getVelocityRadPerSec();
    io.supplyCurrentAmps = armSim.getCurrentDrawAmps();
    io.tempCelsius = 30 + Math.abs(armSim.getCurrentDrawAmps()) * 0.1;
  }

  @Override
  public void updateTunableNumbers() {}

  public void setSimulatedElevatorPosition(double position) {
    this.simulatedElevatorPosition = position;
  }

  @Override
  public void setPosition(double position) {
    if (SafetyChecker.isSafe(
        SafetyChecker.MechanismType.ARM_MOVEMENT, simulatedElevatorPosition, position)) {
      targetPositionRad = position * 2 * Math.PI * drumRadiusMeters / gearing;
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

    armSim.setInputVoltage(power.Output);
  }
}
