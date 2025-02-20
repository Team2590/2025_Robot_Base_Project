package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

public class GamePieceFactory {
  public static Command intakeCoralFeeder() {
    return new ParallelCommandGroup(ArmFactory.setPosition(-0.34), ElevatorFactory.setPosition(42));
  }

  public static Command intakeAlgaeGround() {
    return new ParallelCommandGroup(IntakeFactory.setIntakeAlgaePosition(), IntakeFactory.runIntake(() -> Constants.IntakeConstantsLeonidas.INTAKE_ALGAE_INTAKE_SPEED));
  }

  public static Command intakeCoralGround(){
    return new ParallelCommandGroup(IntakeFactory.setIntakeCoralPosition(), IntakeFactory.runIntake(() -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED));
  }
}
