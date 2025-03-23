package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private LoggedTunableNumber runVoltage =
      new LoggedTunableNumber("Climb/runVoltage", Constants.ClimbConstantsLeonidas.CLIMB_VOLTAGE);
  private DigitalInput toplimitSwitch = new DigitalInput(Constants.ClimbConstantsLeonidas.CLIMB_LIMIT_SWITCH);

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public Command runClimb(double voltage) {
    System.out.println("Running climb");
    return runEnd(() -> io.setVoltage(runVoltage.get()), io::stop);
  }

  public Command resetRotationCount() {
    return runOnce(io::resetRotationCount);
  }

  public double getRotationCount() {
    return inputs.rotationCount;
  }

  public void resetRotationCountFunction() {
    io.resetRotationCount();
  }

  public boolean getClimbLimitSwitchValue(){
    return toplimitSwitch.get();
  }
}
