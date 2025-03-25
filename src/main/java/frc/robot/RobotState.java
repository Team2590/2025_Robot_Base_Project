package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import java.util.HashMap;
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
  private static boolean intakeHasCoral;
  private static boolean intakeHasAlgae;
  private final ControllerOrchestrator controllerApp;

  /** The aligning state for scoring, if we are aligning to front or back of the robot. */
  public static enum AligningState {
    NOT_ALIGNING,
    ALIGNING_FRONT,
    ALIGNING_BACK
  }

  public class ScoringSetpoints {
    public double elevatorSetpoint;
    public double armSetpoint;

    public ScoringSetpoints(double elevatorSetpoint, double armSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.armSetpoint = armSetpoint;
    }
  }

  private AtomicReference<AligningState> aligningState =
      new AtomicReference<RobotState.AligningState>(AligningState.NOT_ALIGNING);

  private static AtomicReference<Pose2d> targetPose = new AtomicReference<Pose2d>(new Pose2d());
  private static AtomicReference<ScoringSetpoints> scoringSetpoints =
      new AtomicReference<ScoringSetpoints>();
  private static HashMap<Level, ScoringSetpoints> levelLookup =
      new HashMap<Level, ScoringSetpoints>();
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
    levelLookup.put(
        Level.L2,
        new ScoringSetpoints(
            Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS,
            Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3));
    levelLookup.put(
        Level.L3,
        new ScoringSetpoints(
            Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS,
            Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3));
    levelLookup.put(
        Level.L4,
        new ScoringSetpoints(
            Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS,
            Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4));
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
    updateLock.lock();
    try {
      setAligningStateBasedOnTargetPose(() -> controllerApp.getTarget().pose());
      updateScoringConfiguration(controllerApp.getTarget().pose());
    } finally {
      updateLock.unlock();
    }
    // currentZone = Constants.locator.getZoneOfField(robotPose);

    endEffectorhasCoral = endEffectorhasCoral();
  }

  /**
   * Checks if endeffector has coral
   *
   * @return true if the endeffector has coral, false if not
   */
  @AutoLogOutput(key = "RobotState/endEffectorHasCoral")
  public static boolean endEffectorhasCoral() {
    return endEffector.hasCoral();
  }

  @AutoLogOutput(key = "RobotState/intakeHasCoral")
  public static boolean intakeHasCoral() {
    return intake.hasCoral();
  }

  /**
   * Checks what zone robot is in
   *
   * @return the zone robot is in
   */
  public String getCurrentZone() {
    return currentZone;
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
    if (drive.frontScore(targetPose.get())) {
      setAligningState(AligningState.ALIGNING_FRONT);
    } else {
      setAligningState(AligningState.ALIGNING_BACK);
    }
    Logger.recordOutput("RobotState/AligningState", getAligningState());
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

  private void updateScoringConfiguration(Pose2d originalTargetPose) {
    ScoringSetpoints lookup = levelLookup.get(controllerApp.getTarget().scoringLevel());
    // I think we need the original controllerApp pose here to avoid evaluating on an already
    // flipped pose (the targetPose of this class)
    // Opted not to use Aligning enum in the event that we attempt to score without aligning
    if (aligningState.get() == AligningState.ALIGNING_BACK) {
      lookup.armSetpoint = Constants.ArmConstantsLeonidas.BACK_HORIZONTAL - lookup.armSetpoint;
      targetPose.set(drive.flipScoringSide(originalTargetPose));
      System.out.println("Scoring Back with an arm setpoint of " + lookup.armSetpoint);
    } else {
      lookup.armSetpoint = lookup.armSetpoint;
      System.out.println("Scoring Front with an arm setpoint of " + lookup.armSetpoint);
    }
    scoringSetpoints.set(lookup);
    Logger.recordOutput("RobotState/Pose", targetPose.get());
    Logger.recordOutput("RobotState/ArmSetpoint", scoringSetpoints.get().armSetpoint);
    Logger.recordOutput("RobotState/ElevatorSetpoint", scoringSetpoints.get().elevatorSetpoint);
  }

  public Pose2d getTargetPose() {
    updateLock.lock();
    try {
      return targetPose.get();
    } finally {
      updateLock.unlock();
    }
  }

  public ScoringSetpoints getScoringSetpoints() {
    updateLock.lock();
    try {
      return scoringSetpoints.get();
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
