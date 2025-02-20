package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class GamePieceFactory {
  public static Command intakeFromFeeder() {
    return new ParallelCommandGroup(ArmFactory.setPosition(-0.34), ElevatorFactory.setPosition(42));
  }
}
