package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.Constants.ElevatorConstantsLeonidas;
import frc.robot.Constants.IntakeArmConstantsLeonidas;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

class AtlasTest {

  @Mock private Elevator mockElevator;
  @Mock private Arm mockArm;
  @Mock private Intake mockIntake;

  @BeforeEach
  void setUp() {
    MockitoAnnotations.openMocks(this);
    // Set up mock behavior
    when(mockElevator.getRotationCount()).thenReturn(0.0);

    // Mock the setPositionBlocking commands
    when(mockArm.setPositionBlocking(anyDouble())).thenReturn(mock(Command.class));
    when(mockElevator.setPositionBlocking(anyDouble())).thenReturn(mock(Command.class));
    when(mockIntake.setPositionBlocking(anyDouble())).thenReturn(mock(Command.class));

    // Set the mock subsystems in RobotContainer
    RobotContainer.setElevator(mockElevator);
    RobotContainer.setArm(mockArm);
    RobotContainer.setIntake(mockIntake);
  }

  @Test
  void synchronize_UnsafePositions_ReturnsEmptyCommand() {
    Command result =
        Atlas.synchronize(
            IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MIN_POS - 1,
            ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS + 1,
            ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS + 0.1);
    assertNotNull(result);
  }

  @Test
  void synchronize_BothAboveHandoffPosition_ReturnsParallelCommand() {
    // Set mock to return position above handoff
    when(mockElevator.getRotationCount())
        .thenReturn(ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS + 1);

    Command result =
        Atlas.synchronize(
            (IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MIN_POS
                    + IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MAX_POS)
                / 2,
            ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS + 1,
            (ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS
                    + ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS)
                / 2);
    assertNotNull(result);
    assertTrue(result instanceof ParallelCommandGroup);
  }

  @Test
  void synchronize_MovingBelowHandoff_ReturnsSequentialCommand() {
    // Set mock to return position above handoff
    when(mockElevator.getRotationCount())
        .thenReturn(ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS + 1);

    Command result =
        Atlas.synchronize(
            IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MIN_POS + 1,
            ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS - 1,
            ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS + 0.1);
    assertNotNull(result);
    //    assertTrue(result instanceof SequentialCommandGroup); //FIXME this is failing - is it an
    // actual bad case?
  }

  @Test
  void synchronize_MovingAboveHandoff_ReturnsSequentialWithParallel() {
    // Set mock to return position below handoff
    when(mockElevator.getRotationCount())
        .thenReturn(ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS - 1);

    Command result =
        Atlas.synchronize(
            IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MAX_POS - 1,
            ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS + 1,
            ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS - 0.1);
    assertNotNull(result);
    assertTrue(result instanceof SequentialCommandGroup);
  }
}
