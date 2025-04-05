package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

import org.apache.commons.math3.analysis.function.Constant;
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
  private double bargeArmPos = 0.175;
  private double processorArmPos = 0;

  private HashMap<String, Double> smartDeAlgaeSetpoints= new HashMap<>();
  
  

  /** The aligning state for scoring, if we are aligning to front or back of the robot. */
  public static enum AligningState {
    NOT_ALIGNING,
    ALIGNING_FRONT,
    ALIGNING_BACK
  }

  public static enum BargeScoringState {
    BARGE_FRONT,
    BARGE_BACK,
  }

  public static enum ProcessorScoringState {
    PROCESSOR_FRONT,
    PROCESSOR_BACK,
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
  private AtomicReference<BargeScoringState> bargeState =
      new AtomicReference<BargeScoringState>(BargeScoringState.BARGE_FRONT);
  private AtomicReference<ProcessorScoringState> processorState =
      new AtomicReference<ProcessorScoringState>(ProcessorScoringState.PROCESSOR_FRONT);
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

    smartDeAlgaeSetpoints.put("N_",  Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2);
    smartDeAlgaeSetpoints.put("NW_", Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3);
    smartDeAlgaeSetpoints.put("NE_", Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3);
    smartDeAlgaeSetpoints.put("NW_", Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3);
    smartDeAlgaeSetpoints.put("SE_", Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2);
    smartDeAlgaeSetpoints.put("SW_", Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2);
    smartDeAlgaeSetpoints.put("S_", Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3);
    




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
      updateScoringConfigurationSimple(
          () -> controllerApp.getTarget().pose(), () -> controllerApp.getTarget().scoringLevel());
    } finally {
      updateLock.unlock();
    }
    if (!endEffector.hasGamePiece()) {
      clearEndEffectorHasGamePiece();
    } else {
      hasGamePiece = true;
    }
    Logger.recordOutput("RobotState/EndEffectorHasGamePiece", hasGamePiece);
    setBargeProcessorAlignment();
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

  public void setBargeProcessorAlignment() {
    Pose2d bargePose = new Pose2d();
    Pose2d processorPose = new Pose2d();

    boolean poseChanged = false;

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        bargePose = new Pose2d(9.96, 2.02, new Rotation2d(-Math.PI));
        processorPose = new Pose2d(11.59, 7.43, new Rotation2d(90));
        poseChanged = true;
      } else {
        bargePose = new Pose2d(7.71, 6.10, new Rotation2d(0));
        processorPose = new Pose2d(6.11, 0.60, new Rotation2d(-90));
        poseChanged = true;
      }
    }

    if (poseChanged) {
      if (drive.frontScore(bargePose)) {
        setBargeState(BargeScoringState.BARGE_FRONT);
      } else {
        setBargeState(BargeScoringState.BARGE_BACK);
      }

      if (drive.frontScore(processorPose)) {
        setProcessorState(ProcessorScoringState.PROCESSOR_FRONT);
      } else {
        setProcessorState(ProcessorScoringState.PROCESSOR_BACK);
      }
    }
  }

  public void resetAligningState() {
    setAligningState(AligningState.NOT_ALIGNING);
  }

  public static boolean endEffectorHasGamePiece() {
    return hasGamePiece;
  }

  public static void clearEndEffectorHasGamePiece() {
    hasGamePiece = false;
  }

  private void updateScoringConfigurationSimple(
      Supplier<Pose2d> originalTargetPose, Supplier<Level> elevatorSetpoint) {
    if (DriverStation.isAutonomous()) return;

    coralScoringSetpoints.elevatorSetpoint = elevatorSetpoint.get().getElevatorSetpoint();

    if (aligningState.get() == AligningState.ALIGNING_FRONT) {
      coralScoringSetpoints.armSetpoint =
          Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L2_PRE; // .15
      coralScoringSetpoints.armPlaceSetpoint =
          Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L2_POST; // 0
      dealgaeSetpoints.armSetpoint = Constants.ArmConstantsLeonidas.ARM_DEALGAE_PRE; // 0
      dealgaeSetpoints.armPlaceSetpoint = Constants.ArmConstantsLeonidas.ARM_DEALGAE_POST; // 0
    } else if (aligningState.get() == AligningState.ALIGNING_BACK) {
      coralScoringSetpoints.armSetpoint =
          Constants.ArmConstantsLeonidas.BACK_HORIZONTAL
              - Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L2_PRE; // .5 - .15
      coralScoringSetpoints.armPlaceSetpoint =
          Constants.ArmConstantsLeonidas.BACK_HORIZONTAL
              - Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L2_POST; // .5 - 0
      dealgaeSetpoints.armSetpoint =
          Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION
              - Constants.ArmConstantsLeonidas.ARM_DEALGAE_PRE; // .5 - 0
      dealgaeSetpoints.armPlaceSetpoint =
          Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION
              - Constants.ArmConstantsLeonidas.ARM_DEALGAE_POST; // .5 - 0
    }

    targetPose = drive.flipScoringSide(originalTargetPose.get());

    if (aligningState.get() == AligningState.ALIGNING_BACK) {
      targetPose = FieldConstants.convertBackScoring(targetPose);
    }

    if (processorState.get() == ProcessorScoringState.PROCESSOR_FRONT) {
      processorArmPos = Constants.ArmConstantsLeonidas.ARM_PROCESSOR_POS;
    } else {
      processorArmPos = Constants.ArmConstantsLeonidas.ARM_PROCESSOR_POS;
    }

    if (bargeState.get() == BargeScoringState.BARGE_FRONT) {
      bargeArmPos = Constants.ArmConstantsLeonidas.ARM_BARGE_POS;
    } else {
      bargeArmPos = Constants.ArmConstantsLeonidas.ARM_BARGE_POS_BACK;
    }

    Logger.recordOutput("RobotState/Pose", targetPose);
    Logger.recordOutput("RobotState/CoralArmSetpoint", coralScoringSetpoints.armSetpoint);
    Logger.recordOutput("RobotState/CoralElevatorSetpoint", coralScoringSetpoints.elevatorSetpoint);

    Logger.recordOutput("RobotState/SmartDealgaeElevatorSetpoint", dealgaeSetpoints.elevatorSetpoint);
    Logger.recordOutput("RobotState/DealgaeArmSetpoint", dealgaeSetpoints.armSetpoint);
    Logger.recordOutput("RobotState/algaePlaceSetpoint", dealgaeSetpoints.armPlaceSetpoint);
    Logger.recordOutput("RobotState/algaeScoringArmSetpoint", algaeScoringSetpoints.armSetpoint);
    Logger.recordOutput(
        "RobotState/algaeScoringPlaceSetpoint", algaeScoringSetpoints.armPlaceSetpoint);
    Logger.recordOutput("RobotState/bargeArmSetpoint", bargeArmPos);
    Logger.recordOutput("RobotState/processorArmSetpoint", processorArmPos);
  }

  public Pose2d getTargetPose() {
    updateLock.lock();
    try {
      return targetPose;
    } finally {
      updateLock.unlock();
    }
  }

  public double getReefOffsetLeft() {

    if (aligningState.get() == AligningState.ALIGNING_BACK) {
      return Drive.reefXOffsetRight.get();
    } else {
      return Drive.reefXOffsetLeft.get();
    }
  }

  public double getReefOffsetRight() {

    if (aligningState.get() == AligningState.ALIGNING_BACK) {

      return Drive.reefXOffsetLeft.get();
    } else {
      return Drive.reefXOffsetRight.get();
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

      String key= controllerApp.getMoveTo();
      for(String side: smartDeAlgaeSetpoints.keySet()){

          if(key.contains(side)){
            setpoint_copy.elevatorSetpoint=smartDeAlgaeSetpoints.get(side);
          }
      }
      //setpoint_copy.elevatorSetpoint = level.getElevatorSetpoint();
      Logger.recordOutput("RobotState/SmartDealgaeAcutal", setpoint_copy.elevatorSetpoint);
      System.out.println("Dealgae Setpoint is+ ", setpoint_copy.elevatorSetpoint);
      return setpoint_copy;
    } finally {
      updateLock.unlock();
    }
  }

  public ScoringSetpoints getAlgaeScoringSetpoints(Level level) {
    updateLock.lock();
    try {
      // Manually set the requested levels elevator setpoint because I am too stupid to figure out a
      // better wayf
      ScoringSetpoints setpoint_copy = algaeScoringSetpoints;
      setpoint_copy.elevatorSetpoint = level.getElevatorSetpoint();
      return setpoint_copy;
    } finally {
      updateLock.unlock();
    }
  }

  public ProcessorScoringState getProcessorState() {
    return processorState.get();
  }

  public BargeScoringState getBargeState() {
    return bargeState.get();
  }

  public void setProcessorState(ProcessorScoringState state) {
    processorState.set(state);
  }

  public void setBargeState(BargeScoringState state) {
    bargeState.set(state);
  }

  public double getProcessorArmPos() {
    updateLock.lock();
    try {
      return processorArmPos;
    } finally {
      updateLock.unlock();
    }
  }

  public double getBargeArmPos() {
    updateLock.lock();
    try {
      return bargeArmPos;
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
