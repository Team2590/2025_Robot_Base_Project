package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.util.NemesisMathUtil;
import java.util.function.Supplier;

public class AutoFactory {
  public static Command driveTo(Pose2d target) {
    return DriveCommands.driveToPose(target)
        .onlyIf(() -> NemesisMathUtil.distance(target, RobotContainer.getDrive().getPose()) > 1)
        .andThen(
            DriveCommands.driveToPoseStraight(RobotContainer.getDrive(), () -> target)
                .until(
                    () ->
                        NemesisMathUtil.isPoseApprox(
                            RobotContainer.getDrive().getPose(), target, 0.02)));
  }

  public static Command driveTo(Supplier<Pose2d> target) {
    return driveTo(target.get());
  }
}
