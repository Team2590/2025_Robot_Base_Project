package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState extends SubsystemBase {
  Pose2d robotPose;
  private String currentZone;
  private final Arm arm;
  private final Drive drive;
  private final Elevator elevator;
  private final Vision vision;
  private static EndEffector endEffector;
  private static Intake intake;
  private static RobotState instance;
  @Getter private static boolean endEffectorhasCoral;
  private static boolean intakeHasCoral;
  private static boolean intakeHasAlgae;

  /** The aligning state for scoring, if we are aligning to front or back of the robot. */
  public static enum AligningState {
    NOT_ALIGNING,
    ALIGNING_FRONT,
    ALIGNING_BACK
  }

  private AtomicReference<AligningState> aligningState =
      new AtomicReference<RobotState.AligningState>(AligningState.NOT_ALIGNING);

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

  /**
   * Initializes the robot state
   *
   * @param arm robots arm
   * @param drive drive
   * @param elevator elevator
   * @param endEffector endeffector
   * @param intake intake
   * @param vision vision
   * @return new robot state object
   */
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

  /**
   * Gets the Robot State if it's initialized
   *
   * @return robot state
   */
  public static RobotState getInstance() {
    if (instance == null) {
      throw new IllegalStateException("RobotState has not been initialized");
    }
    return instance;
  }

  /**
   * Gets Robot Pose
   *
   * @return robot pose
   */
  public Pose2d getPose() {
    return drive.getPose();
  }

  public void periodic() {
    robotPose = drive.getPose();
    // currentZone = Constants.locator.getZoneOfField(robotPose);
    endEffectorhasCoral = endEffectorhasCoral();
  }

  /**
   * Checks if endeffector has coral
   *
   * @return true if the endeffector has coral, false if not
   */
  @AutoLogOutput(key = "EndEffector/hasCoral")
  public static boolean endEffectorhasCoral() {
    return endEffector.hasCoral();
  }

  @AutoLogOutput(key = "Intake/hasCoral")
  public static boolean intakeHasCoral() {
    return intake.hasCoral();
  }

  /**
   * Checks if intake has algae
   *
   * @return true if the intake has algae, false if not
   */
  @AutoLogOutput(key = "Intake/hasAlgae")
  public static boolean intakeHasAlgae() {
    return intake.hasAlgae();
  }

  /**
   * Checks what zone robot is in
   *
   * @return the zone robot is in
   */
  public String getCurrentZone() {
    return currentZone;
  }

  public Pose2d getNearestCoralPose() {
    return vision.getNearestCoralPose();
  }

  @AutoLogOutput(key = "PreciseAlignment/AligningState")
  public AligningState getAligningState() {
    return aligningState.get();
  }

  public void setAligningState(AligningState state) {
    aligningState.set(state);
  }

  /** Align to the front or back of the robot based on the given target pose. */
  public void setAligningStateBasedOnTargetPose(Supplier<Pose2d> targetPose) {
    Logger.recordOutput("PreciseAlignment/TargetPose", targetPose.get());
    if (drive.frontScore(targetPose.get())) {
      setAligningState(AligningState.ALIGNING_FRONT);
    } else {
      setAligningState(AligningState.ALIGNING_BACK);
    }
  }

  public void resetAligningState() {
    setAligningState(AligningState.NOT_ALIGNING);
  }

  public static Command setIntakeHasCoral() {
    return Commands.runOnce(() -> intakeHasCoral = true);
  }

  public static Command setIntakeNoCoral() {
    return Commands.runOnce(() -> intakeHasCoral = false);
  }

  public static Command setIntakeHasAlgae() {
    return Commands.runOnce(() -> intakeHasAlgae = true);
  }

  public static Command setIntakeNoAlgae() {
    return Commands.runOnce(() -> intakeHasAlgae = false);
  }
}
