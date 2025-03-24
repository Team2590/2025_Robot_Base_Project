package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.Constants.ElevatorConstantsLeonidas;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class RobotStateTest {

  @Mock private Arm arm;
  @Mock private Drive drive;
  @Mock private Elevator elevator;
  @Mock private EndEffector endEffector;
  @Mock private Intake intake;
  @Mock private Vision vision;
  @Mock private ControllerOrchestrator controllerApp;

  private RobotState robotState;
  private MockedStatic<RobotContainer> mockedRobotContainer;

  @BeforeEach
  void setUp() {
    // Reset the singleton instance before each test
    try {
      Field instance = RobotState.class.getDeclaredField("instance");
      instance.setAccessible(true);
      instance.set(null, null);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Initialize static mocking
    mockedRobotContainer = Mockito.mockStatic(RobotContainer.class);
    when(RobotContainer.getArm()).thenReturn(arm);
    when(RobotContainer.getElevator()).thenReturn(elevator);

    robotState =
        RobotState.initialize(arm, drive, elevator, endEffector, intake, vision, controllerApp);
  }

  @AfterEach
  void tearDown() {
    // Clean up the singleton instance after each test
    try {
      Field instance = RobotState.class.getDeclaredField("instance");
      instance.setAccessible(true);
      instance.set(null, null);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Close static mocking
    mockedRobotContainer.close();
  }

  @Test
  void testScoringSetpointsCreation() {
    // Verify that ScoringSetpoints correctly stores and retrieves elevator and arm setpoints
    RobotState.ScoringSetpoints setpoints = new RobotState.ScoringSetpoints(10.0, 20.0);
    assertEquals(10.0, setpoints.getElevatorSetpoint());
    assertEquals(20.0, setpoints.getArmSetpoint());
  }

  @Test
  void testScoringSetpointsCalculation() {
    // Verify that scoring setpoints are correctly calculated for front scoring at L3
    // This includes checking both elevator height and arm angle for the specified level
    when(controllerApp.getTarget())
        .thenReturn(
            new ControllerOrchestrator.Target(
                new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d()), Level.L3));

    when(drive.frontScore(any())).thenReturn(true);

    robotState.setScoringSetpoints();
    RobotState.ScoringSetpoints setpoints = robotState.getScoringSetpoints();

    assertNotNull(setpoints);
    assertEquals(ElevatorConstantsLeonidas.ELEVATOR_L3_POS, setpoints.getElevatorSetpoint());
    assertEquals(ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3, setpoints.getArmSetpoint());
  }

  @Test
  void testEndEffectorStateTracking() {
    // Verify that the end effector's coral state is correctly tracked and updated
    // This includes checking both the presence and absence of coral
    when(endEffector.hasCoral()).thenReturn(true);
    when(controllerApp.getTarget())
        .thenReturn(
            new ControllerOrchestrator.Target(
                new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d()), Level.L3));
    when(drive.flipScoringSide(any())).thenAnswer(invocation -> invocation.getArgument(0));

    robotState.periodic();
    assertTrue(RobotState.endEffectorhasCoral());

    when(endEffector.hasCoral()).thenReturn(false);
    robotState.periodic();
    assertFalse(RobotState.endEffectorhasCoral());
  }

  @Test
  void testTargetPoseSetting() {
    // Verify that target poses can be set and retrieved correctly
    // This ensures the robot can track its intended destination
    Pose2d testPose = new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d());
    when(drive.flipScoringSide(any())).thenAnswer(invocation -> invocation.getArgument(0));
    robotState.setTargetPose(testPose);
    assertEquals(testPose, robotState.getTargetPose());
  }

  @Test
  void testNullTargetPose() {
    // Verify that the system properly handles null target poses
    // This ensures robust error handling when no target is specified
    when(controllerApp.getTarget()).thenReturn(null);
    assertThrows(
        NullPointerException.class,
        () -> {
          robotState.periodic();
        });
  }

  @Test
  void testThreadSafety() throws InterruptedException {
    // Verify that target pose updates are thread-safe
    // This ensures concurrent updates don't corrupt the robot's state
    Pose2d testPose1 = new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d());
    Pose2d testPose2 = new Pose2d(new Translation2d(3.0, 4.0), new Rotation2d());

    when(controllerApp.getTarget())
        .thenReturn(new ControllerOrchestrator.Target(testPose1, Level.L3))
        .thenReturn(new ControllerOrchestrator.Target(testPose2, Level.L3));
    when(drive.flipScoringSide(any())).thenAnswer(invocation -> invocation.getArgument(0));

    Thread thread1 = new Thread(() -> robotState.periodic());
    Thread thread2 = new Thread(() -> robotState.periodic());

    thread1.start();
    thread2.start();

    thread1.join();
    thread2.join();

    assertEquals(testPose2, robotState.getTargetPose());
  }

  @Test
  void testAllianceBasedPositionFlipping() {
    // Verify that scoring positions are correctly flipped based on alliance
    // This ensures the robot can score from either side of the field
    when(controllerApp.getTarget())
        .thenReturn(
            new ControllerOrchestrator.Target(
                new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d()), Level.L3));

    when(drive.frontScore(any())).thenReturn(false);

    robotState.setScoringSetpoints();
    RobotState.ScoringSetpoints setpoints = robotState.getScoringSetpoints();

    assertNotNull(setpoints);
    assertEquals(ElevatorConstantsLeonidas.ELEVATOR_L3_POS, setpoints.getElevatorSetpoint());
    assertEquals(
        ArmConstantsLeonidas.BACK_HORIZONTAL - ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3,
        setpoints.getArmSetpoint());
  }

  @Test
  void testScoringSetpointsForDifferentLevels() {
    // Verify that scoring setpoints are correctly calculated for different scoring heights
    // This ensures the robot can score at all required levels (L2, L3, L4)
    when(controllerApp.getTarget())
        .thenReturn(
            new ControllerOrchestrator.Target(
                new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d()), Level.L2));
    when(drive.frontScore(any())).thenReturn(true);
    robotState.setScoringSetpoints();
    RobotState.ScoringSetpoints l2Setpoints = robotState.getScoringSetpoints();
    assertEquals(ElevatorConstantsLeonidas.ELEVATOR_L2_POS, l2Setpoints.getElevatorSetpoint());
    assertEquals(ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3, l2Setpoints.getArmSetpoint());

    when(controllerApp.getTarget())
        .thenReturn(
            new ControllerOrchestrator.Target(
                new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d()), Level.L4));
    robotState.setScoringSetpoints();
    RobotState.ScoringSetpoints l4Setpoints = robotState.getScoringSetpoints();
    assertEquals(ElevatorConstantsLeonidas.ELEVATOR_L4_POS, l4Setpoints.getElevatorSetpoint());
    assertEquals(ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4, l4Setpoints.getArmSetpoint());
  }
}
