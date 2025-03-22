package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.util.NemesisMathUtil;
import java.util.function.Supplier;

public class AutoFactory {
  public static Command driveTo(Pose2d target) {
    Pose2d actualTarget = RobotContainer.getDrive().flipScoringSide(target);

    // spotless:off
    return DriveCommands.driveToPose(actualTarget)
    .until(() -> NemesisMathUtil.distance(RobotContainer.getDrive().getPose(), actualTarget) < 1)
    .andThen(
      DriveCommands.driveToPoseStraight(RobotContainer.getDrive(), () -> actualTarget)
      .until(() -> 
        actualTarget.getTranslation().getDistance(RobotContainer.getDrive().getPose().getTranslation()) < 0.02 && 
        actualTarget.getRotation().minus(RobotContainer.getDrive().getPose().getRotation()).getDegrees() < 4
      )
    );
    // spotless:on
  }

  public static Command driveTo(Supplier<Pose2d> target) {
    return driveTo(target.get());
  }
}
