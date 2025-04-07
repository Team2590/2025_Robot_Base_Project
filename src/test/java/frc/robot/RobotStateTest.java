package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.RobotState.ScoringSetpoints;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;

class RobotStateTest {
  @Mock private Arm arm;
  @Mock private Drive drive;
  @Mock private Elevator elevator;
  @Mock private EndEffector endEffector;
  @Mock private Intake intake;
  @Mock private Vision vision;
  @Mock private ControllerOrchestrator controllerApp;
  @Mock private ControllerOrchestrator.Target target;

  private RobotState robotState;
  private AutoCloseable mockCloseable;
  private MockedStatic<DriverStation> driverStationMock;
  private MockedStatic<RobotContainer> robotContainerMock;

  @BeforeEach
  void setUp() {
    mockCloseable = MockitoAnnotations.openMocks(this);
    driverStationMock = Mockito.mockStatic(DriverStation.class);
    robotContainerMock = Mockito.mockStatic(RobotContainer.class);

    // Reset the singleton instance
    try {
      var field = RobotState.class.getDeclaredField("instance");
      field.setAccessible(true);
      field.set(null, null);
    } catch (Exception e) {
      throw new RuntimeException("Failed to reset RobotState singleton", e);
    }

    // Mock the target chain
    when(controllerApp.getTarget()).thenReturn(target);
    when(target.pose()).thenReturn(new Pose2d(0, 0, new Rotation2d(0)));
    when(target.scoringLevel()).thenReturn(Level.L2);

    // Mock DriverStation
    driverStationMock.when(DriverStation::isAutonomous).thenReturn(false);

    // Mock RobotContainer static methods
    robotContainerMock.when(RobotContainer::getDrive).thenReturn(drive);
    robotContainerMock.when(RobotContainer::getArm).thenReturn(arm);
    robotContainerMock.when(RobotContainer::getElevator).thenReturn(elevator);

    // Mock drive pose and methods
    when(drive.getPose()).thenReturn(new Pose2d(0, 0, new Rotation2d(0)));
    when(drive.flipScoringSide(any(Pose2d.class)))
        .thenAnswer(invocation -> invocation.getArgument(0));

    robotState =
        RobotState.initialize(arm, drive, elevator, endEffector, intake, vision, controllerApp);
  }

  @AfterEach
  void tearDown() throws Exception {
    mockCloseable.close();
    driverStationMock.close();
    robotContainerMock.close();
  }

  @Test
  void updateScoringConfigurationSimple_FrontAlignment_ArmBelowHandoff_ReturnsCorrectSetpoints() {
    // Arrange
    robotState.setAligningState(RobotState.AligningState.ALIGNING_FRONT);
    System.out.println(robotState.getAligningState().name());
    when(arm.getAbsolutePosition()).thenReturn(ArmConstantsLeonidas.ARM_HANDOFF_POS - 0.1);

    // Act
    robotState.periodic(); // This will call updateScoringConfigurationSimple
    System.out.println(robotState.getAligningState().name());

    // Assert
    ScoringSetpoints setpoints = robotState.getCoralScoringSetpoints();
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_FRONT_FRONT_PRE, setpoints.armSetpoint);
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_FRONT_FRONT_POST, setpoints.armPlaceSetpoint);
    assertEquals(Level.L2.getElevatorSetpoint(), setpoints.elevatorSetpoint);
  }

  @Test
  void updateScoringConfigurationSimple_FrontAlignment_ArmAboveBackFront_ReturnsCorrectSetpoints() {
    // Arrange
    robotState.setAligningState(RobotState.AligningState.ALIGNING_FRONT);
    when(arm.getAbsolutePosition()).thenReturn(ArmConstantsLeonidas.ARM_SCORE_BACK_FRONT_PRE + 0.1);

    // Act
    robotState.periodic();

    // Assert
    ScoringSetpoints setpoints = robotState.getCoralScoringSetpoints();
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_BACK_FRONT_PRE, setpoints.armSetpoint);
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_BACK_FRONT_POST, setpoints.armPlaceSetpoint);
    assertEquals(Level.L2.getElevatorSetpoint(), setpoints.elevatorSetpoint);
  }

  @Test
  void updateScoringConfigurationSimple_BackAlignment_ArmBelowFrontBack_ReturnsCorrectSetpoints() {
    // Arrange
    robotState.setAligningState(RobotState.AligningState.ALIGNING_BACK);
    when(arm.getAbsolutePosition()).thenReturn(ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_PRE - 0.1);

    // Act
    robotState.periodic();

    // Assert
    ScoringSetpoints setpoints = robotState.getCoralScoringSetpoints();
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_PRE, setpoints.armSetpoint);
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_POST, setpoints.armPlaceSetpoint);
    assertEquals(Level.L2.getElevatorSetpoint(), setpoints.elevatorSetpoint);
  }

  @Test
  void updateScoringConfigurationSimple_BackAlignment_ArmAboveHandoff_ReturnsCorrectSetpoints() {
    // Arrange
    robotState.setAligningState(RobotState.AligningState.ALIGNING_BACK);
    when(arm.getAbsolutePosition()).thenReturn(ArmConstantsLeonidas.ARM_HANDOFF_POS + 0.1);

    // Act
    robotState.periodic();

    // Assert
    ScoringSetpoints setpoints = robotState.getCoralScoringSetpoints();
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_BACK_BACK_PRE, setpoints.armSetpoint);
    assertEquals(ArmConstantsLeonidas.ARM_SCORE_BACK_BACK_POST, setpoints.armPlaceSetpoint);
    assertEquals(Level.L2.getElevatorSetpoint(), setpoints.elevatorSetpoint);
  }

  @Test
  void updateScoringConfigurationSimple_AutonomousMode_AlwaysSetsFrontAlignment() {
    // Arrange
    driverStationMock.when(DriverStation::isAutonomous).thenReturn(true);
    robotState.setAligningState(RobotState.AligningState.ALIGNING_BACK);

    // Act
    robotState.periodic();

    // Assert
    assertEquals(RobotState.AligningState.ALIGNING_FRONT, robotState.getAligningState());
  }
}
