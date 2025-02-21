package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PolygonLocator;
import frc.robot.FieldConstants;

public class RobotState extends SubsystemBase{
    Alliance alliance;
    Pose2d robotPose;
    private String currentZone;
    private EndEffector endEffector = RobotContainer.getEndEffector();
    private Intake intake = RobotContainer.getIntake();
    

    public void periodic() {
        alliance = DriverStation.getAlliance().get();
        robotPose = RobotContainer.getDrive().getPose();
        currentZone = Constants.locator.getZoneOfField(robotPose);
    }

    public boolean hasCoral() {
        return endEffector.hasCoral();
    }

    public boolean hasAlgae() {
        return intake.hasAlgae();
    }

    public boolean intakeRunning() {
        return intake.isRunning();
    }

    public String getCurrentZone() {
        return currentZone;
    }

}
