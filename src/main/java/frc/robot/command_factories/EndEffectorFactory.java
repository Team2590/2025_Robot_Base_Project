package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class EndEffectorFactory {
    private static RobotContainer container = Robot.getRobotContainerInstance();

    /**
     * Creates a command to run the endeffector intake.
     *
     * @param container The RobotContainer instance
     * @param intakeSpeed Supplier for the intake speed
     * @return Command to run the intake
     */
    public static Command runEndEffector() {
        return container.getEndEffector().runEndEffector().withName("Run Endeffector");
    }

        /**
     * Creates a command to run the endeffector outtake.
     *
     * @param container The RobotContainer instance
     * @param intakeSpeed Supplier for the intake speed
     * @return Command to run the intake
     */
    public static Command runEndEffectorOuttake() {
        return container.getEndEffector().runEndEffectorOuttake().withName("Run Endeffector");
    } 
}
