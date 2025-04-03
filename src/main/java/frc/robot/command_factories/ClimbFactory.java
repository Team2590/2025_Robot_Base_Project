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
  // public static Command finishClimb() {
  //   return RobotContainer.getClimb()
  //       .runClimb(Constants.ClimbConstantsLeonidas.CLIMB_VOLTAGE)
  //       .onlyIf(() -> RobotContainer.getClimb().getLimitSwitchValue())
  //       .until(
  //           () ->
  //               RobotContainer.getClimb().getRotationCount()
  //                   > Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION)
  //       .withName("Finish climb");
  // }

  // public static Command deployMechanism(){
  //   return RobotContainer.getClimb()
  //   .runClimb(Constants.ClimbConstantsLeonidas.CLIMB_VOLTAGE)
  //   .until(() -> RobotContainer.getClimb().getLimitSwitchValue())
  //   .withName("Run climb");
  // }
