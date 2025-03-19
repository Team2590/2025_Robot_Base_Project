package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.util.NemesisMathUtil;

public class AutoFactory {
  private static Command intakeCoralAroundSource(Pose2d redPose, Pose2d bluePose) {
    if (DriverStation.getAlliance().isPresent()) {
      // return Commands.defer(
      //     () -> {
      //       return DriverStation.getAlliance().get() == Alliance.Red
      //           ? driveTo(redPose)
      //           : driveTo(bluePose);
      //     },
      //     Set.of(RobotContainer.getDrive()));
      return driveTo(bluePose);
    }
    return Commands.none();
  }

  public static Command intakeCoralAroundSourceRight() {
    return intakeCoralAroundSource(
        FieldConstants.redSourceRightIntakePose, FieldConstants.blueSourceRightIntakePose);
  }

  public static Command intakeCoralAroundSourceLeft() {
    return intakeCoralAroundSource(
        FieldConstants.redSourceLeftIntakePose, FieldConstants.blueSourceLeftIntakePose);
  }

  public static Command driveTo(Pose2d target) {
    // return DriveCommands.preciseAlignment(
    //         RobotContainer.getDrive(), () -> target, target.getRotation())
    //     .until(
    //         () -> NemesisMathUtil.isPoseApprox(target, RobotContainer.getDrive().getPose(),
    // 0.02));
    return DriveCommands.driveToPose(target)
        .andThen(
            DriveCommands.driveToPoseStraight(RobotContainer.getDrive(), () -> target)
                .until(
                    () ->
                        NemesisMathUtil.isPoseApprox(
                            RobotContainer.getDrive().getPose(), target, 0.02)));
  }
}
