package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.Constants.ElevatorConstantsLeonidas;
import frc.robot.RobotState.ScoringSetpoints;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.NemesisMathUtil;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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
  private static boolean hasGamePiece;
  private final ControllerOrchestrator controllerApp;

  private static Pose2d targetPose = new Pose2d();
  private ScoringSetpoints coralScoringSetpoints =
      new ScoringSetpoints(
          Level.L2.getElevatorSetpoint(),
          Level.L2.getarmPreScoreSetpoint(),
          Level.L2.getArmScoringSetpoint());
  private ScoringSetpoints algaeScoringSetpoints =
      new ScoringSetpoints(
          Level.BARGE.getElevatorSetpoint(),
          Level.BARGE.getarmPreScoreSetpoint(),
          Level.BARGE.getArmScoringSetpoint());
  private ScoringSetpoints dealgaeSetpoints =
      new ScoringSetpoints(
          Level.DEALGAE_L2.getElevatorSetpoint(),
          Level.DEALGAE_L2.getarmPreScoreSetpoint(),
          Level.DEALGAE_L2.getArmScoringSetpoint());

  /** The aligning state for scoring, if we are aligning to front or back of the robot. */
  public static enum AligningState {
    NOT_ALIGNING,
    ALIGNING_FRONT,
    ALIGNING_BACK
  }

  /** The aligning state for what side we are scoring on. */
  public static enum AligningSide {
    LEFT,
    RIGHT
  }

  public static class ScoringSetpoints {
    public double elevatorSetpoint;
    public double armSetpoint;
    public double armPlaceSetpoint;

    public ScoringSetpoints(double elevatorSetpoint, double armSetpoint, double armPlaceSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.armSetpoint = armSetpoint;
      this.armPlaceSetpoint = armPlaceSetpoint;
    }
  }

  private AtomicReference<AligningState> aligningState =
      new AtomicReference<RobotState.AligningState>(AligningState.NOT_ALIGNING);
  private AtomicReference<AligningSide> aligningSide =
      new AtomicReference<AligningSide>(AligningSide.LEFT);
  private AligningState previousAligningState = AligningState.NOT_ALIGNING;
  private final Lock updateLock = new ReentrantLock();

  private RobotState(
      Arm arm,
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      Intake intake,
      Vision vision,
      ControllerOrchestrator controllerApp) {
    this.arm = arm;
    this.drive = drive;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.intake = intake;
    this.vision = vision;
    this.controllerApp = controllerApp;
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
      Vision vision,
      ControllerOrchestrator controllerApp) {
    if (instance != null) {
      throw new IllegalStateException("RobotState has already been initialized");
    }
    instance = new RobotState(arm, drive, elevator, endEffector, intake, vision, controllerApp);
    return instance;
  }

  /**
   * Gets the Robot State if it's initialized
   *
   * @return robot state
   */
  public static RobotState getInstance() {
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
    updateLock.lock();
    try {
      setAligningStateBasedOnTargetPose(() -> controllerApp.getTarget().pose());
      updateScoringConfiguration(() -> controllerApp.getTarget().pose());
    } finally {
      updateLock.unlock();
    }
    if (!endEffector.hasGamePiece()) {
      clearEndEffectorHasGamePiece();
    } else {
      hasGamePiece = true;
    }
    Logger.recordOutput("RobotState/EndEffectorHasGamePiece", hasGamePiece);
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

  @AutoLogOutput(key = "PreciseAlignment/AligningSide")
  public AligningSide getAligningSide() {
    return aligningSide.get();
  }

  public void setAligningState(AligningState state) {
    aligningState.set(state);
  }

  /** Align to the front or back of the robot based on the given target pose. */
  public void setAligningStateBasedOnTargetPose(Supplier<Pose2d> targetPose) {
    if (drive.frontScore(targetPose.get())) {
      setAligningState(AligningState.ALIGNING_FRONT);
    } else {
      setAligningState(AligningState.ALIGNING_BACK);
    }
    Logger.recordOutput("RobotState/AligningState", getAligningState());
    if (ControllerOrchestrator.aligningLeft) {
      aligningSide.set(AligningSide.LEFT);
    } else {
      aligningSide.set(AligningSide.RIGHT);
    }
  }

  public void resetAligningState() {
    setAligningState(AligningState.NOT_ALIGNING);
  }

  public void setAligningSide(boolean isLeft) {
    if (isLeft) {
      aligningSide.set(AligningSide.LEFT);
    } else {
      aligningSide.set(AligningSide.RIGHT);
    }
  }

  public static boolean endEffectorHasGamePiece() {
    return hasGamePiece;
  }

  public static void clearEndEffectorHasGamePiece() {
    hasGamePiece = false;
  }

  private void updateScoringConfiguration(Supplier<Pose2d> originalTargetPose) {
    AligningState currentAligningState = aligningState.get();

    if (currentAligningState != previousAligningState) {
      double offset = 0;
      double magnitude = 1;

      if (currentAligningState == AligningState.ALIGNING_BACK) {
        offset = Constants.ArmConstantsLeonidas.BACK_HORIZONTAL;
        magnitude = -1;
      }
      coralScoringSetpoints.armSetpoint =  magnitude * coralScoringSetpoints.armSetpoint + offset;
      coralScoringSetpoints.armPlaceSetpoint =
          magnitude * coralScoringSetpoints.armPlaceSetpoint + offset;

      // update algae setpoints
      dealgaeSetpoints.armSetpoint = magnitude * dealgaeSetpoints.armSetpoint + offset;
      dealgaeSetpoints.armPlaceSetpoint = magnitude * dealgaeSetpoints.armPlaceSetpoint + offset;

      // Update algae scoring setpoints
      algaeScoringSetpoints.armSetpoint = magnitude * algaeScoringSetpoints.armSetpoint + offset;
      algaeScoringSetpoints.armPlaceSetpoint =
          magnitude * algaeScoringSetpoints.armPlaceSetpoint + offset;
      targetPose = drive.flipScoringSide(originalTargetPose.get());

      // Update the previous state
      previousAligningState = currentAligningState;

      Logger.recordOutput("RobotState/Pose", targetPose);
      Logger.recordOutput("RobotState/CoralArmSetpoint", coralScoringSetpoints.armSetpoint);
      Logger.recordOutput(
          "RobotState/CoralArmPlaceSetpoint", coralScoringSetpoints.armPlaceSetpoint);
      Logger.recordOutput("RobotState/algaeArmSetpoint", dealgaeSetpoints.armSetpoint);
      Logger.recordOutput("RobotState/algaePlaceSetpoint", dealgaeSetpoints.armPlaceSetpoint);
      Logger.recordOutput("RobotState/algaeScoringArmSetpoint", algaeScoringSetpoints.armSetpoint);
      Logger.recordOutput(
          "RobotState/algaeScoringPlaceSetpoint", algaeScoringSetpoints.armPlaceSetpoint);
    }
  }

  public Pose2d getTargetPose() {
    updateLock.lock();
    try {
      return targetPose;
    } finally {
      updateLock.unlock();
    }
  }

  public ScoringSetpoints getCoralScoringSetpoints() {
    updateLock.lock();
    try {
      return coralScoringSetpoints;
    } finally {
      updateLock.unlock();
    }
  }

  public ScoringSetpoints getDealgaeSetpoints(Level level) {
    updateLock.lock();
    try {
      // Manually set the requested levels elevator setpoint because I am too stupid to figure out a
      // better way
      ScoringSetpoints setpoint_copy = dealgaeSetpoints;
      setpoint_copy.elevatorSetpoint = level.getElevatorSetpoint();
      return setpoint_copy;
    } finally {
      updateLock.unlock();
    }
  }

  public ScoringSetpoints getAlgaeScoringSetpoints(Level level) {
    updateLock.lock();
    try {
      // Manually set the requested levels elevator setpoint because I am too stupid to figure out a
      // better way
      ScoringSetpoints setpoint_copy = algaeScoringSetpoints;
      setpoint_copy.elevatorSetpoint = level.getElevatorSetpoint();
      return setpoint_copy;
    } finally {
      updateLock.unlock();
    }
  }

  /*
   * Helper Function, picks the closest value for the arm setpoint based on if the cancoder is wrapped or not
   */
  public static double wrapArmSetpoint(double setpoint) {

    double current = RobotContainer.getArm().getAbsolutePosition();
    double wrappedSetpoint = setpoint + ArmConstantsLeonidas.ARM_WRAP_POS;
    // Only wrap if we are able to freely rotate without fear of collision
    if (RobotContainer.getElevator().getRotationCount()
        >= ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS) {
      return NemesisMathUtil.selectClosest(setpoint, wrappedSetpoint, current);
    } else {
      return setpoint;
    }
  }
}
