package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbFactory {
  public static Command runClimb(double position) {
    return RobotContainer.getClimb()
        .runClimb(Constants.ClimbConstantsLeonidas.CLIMB_VOLTAGE)
        .until(() -> RobotContainer.getClimb().getRotationCount() > position)
        .withName("Run climb");
  }
}
