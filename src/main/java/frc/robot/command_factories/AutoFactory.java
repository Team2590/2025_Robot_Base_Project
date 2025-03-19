package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.util.NemesisMathUtil;

public class AutoFactory {
    private static Command intakeCoralAroundSource(Pose2d redPose, Pose2d bluePose) {
        return DriverStation.getAlliance()
                .map(
                    alliance -> driveTo(alliance == Alliance.Red ? redPose : bluePose)
                    .andThen(GamePieceFactory.intakeCoralGround()))
                    .orElse(new SequentialCommandGroup(Commands.none())
                );
    }
    
    public static Command intakeCoralAroundSourceRight() {
        return intakeCoralAroundSource(FieldConstants.redSourceRightIntakePose, FieldConstants.blueSourceRightIntakePose);
    }
    
    public static Command intakeCoralAroundSourceLeft() {
        return intakeCoralAroundSource(FieldConstants.redSourceLeftIntakePose, FieldConstants.blueSourceLeftIntakePose);
    }

    public static Command driveTo(Pose2d target) { 
        return DriveCommands.preciseAlignment(
            RobotContainer.getDrive(), 
            () -> target, 
            target.getRotation()
        ).until(() -> NemesisMathUtil.isPoseApprox(target, RobotContainer.getDrive().getPose(), 0.02));
    }
}
