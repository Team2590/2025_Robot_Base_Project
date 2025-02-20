package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;

public class ClimbFactory {
    public static Command runClimb(double position) {
        return RobotContainer.getClimb().runClimb(0.05).withName("Run climb")
                .onlyIf(() -> RobotContainer.getClimb()
                        .getRotationCount() < position);
    }
}
