package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SafetyChecker;
import frc.robot.util.SafetyChecker.MechanismType;

public class IntakeArmIOSim implements IntakeArmIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0;
  private double position = 0; // Track simulated position

  public IntakeArmIOSim(DCMotor motorModel, double reduction, double inertia) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, inertia, reduction), motorModel);
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    // Simulation doesn't inherently need neutral mode, but you might use it
    // for internal tracking if needed.  For now, just print a warning.
    // System.out.println("IntakeArmIOSim: setNeutralMode called, but has no effect in
    // simulation.");
  }

  @Override
  public void updateInputs(IntakeArmIOInputs io) {
    sim.setInputVoltage(appliedVoltage); // Set voltage to the sim
    sim.update(0.02); // Simulate for 20ms (adjust as needed)
  }

  @Override
  public void updateTunableNumbers() {
    // If you have tunable PID gains or other parameters, update them here
    // from preferences or network tables. For simulation, it might be empty.
  }

  @Override
  public void setPosition(double position) {
    if (SafetyChecker.isSafe(MechanismType.INTAKE_MOVEMENT, position)) {
      this.position = position;
      // Set the internal position.  In a real system, this would likely involve
      // closed-loop control.
      // In a simulation, we're directly setting the position, which is less realistic.
      // A more accurate simulation would apply a voltage to reach the target position.
      //  That would require a more sophisticated model.
    } else {
      // System.out.println("CAN'T MOVE INTAKE ARM, SAFETY CHECK FAILED");
    }
    // System.out.println(
    //     "IntakeArmIOSim: setPosition called directly.  This is a simplified simulation
    // behavior.");
  }

  @Override
  public void resetRotationCount() {
    position = 0; // Resets sim state, including velocity.
  }

  @Override
  public void stop() {
    appliedVoltage = 0;
  }

  @Override
  public double getRotationCount() {
    return sim.getAngularPositionRotations();
  }

  @Override
  public void setInitPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInitPosition'");
  }
}
