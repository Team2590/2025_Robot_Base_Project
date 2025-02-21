package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PolygonLocator;
import frc.robot.FieldConstants;

public class RobotState extends SubsystemBase{
    Alliance alliance;
    Pose2d robotPose;
    private String currentZone;
    private final Arm arm;
    private final Drive drive;
    private final Elevator elevator;
    private final Vision vision;
    private final EndEffector endEffector;
    private final Intake intake;
    private static RobotState instance;


    private RobotState(
      Arm arm,
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      Intake intake,
      Vision vision) {
    this.arm = arm;
    this.drive = drive;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.intake = intake;
    this.vision = vision;
  }
    
  public static RobotState initialize(
    Arm arm,
    Drive drive,
    Elevator elevator,
    EndEffector endEffector,
    Intake intake,
    Vision vision) {
        if (instance != null) {
            throw new IllegalStateException("RobotState has already been initialized");
        }
        instance = new RobotState(arm, drive, elevator, endEffector, intake, vision);
        return instance;
    }

    public static RobotState getInstance() {
        if (instance == null) {
          throw new IllegalStateException("RobotState has not been initialized");
        }
        return instance;
    }

    public Pose2d getPose() {
        return drive.getPose();
    }  
  
    public void periodic() {
        alliance = DriverStation.getAlliance().get();
        robotPose = drive.getPose();
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
