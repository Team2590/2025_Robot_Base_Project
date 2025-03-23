package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DefaultCommandsTest {
  private static CommandScheduler scheduler;
  private static RobotContainer robotContainer;

  @BeforeAll // this method will run once before all tests
  static void initialize() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    FieldConstants.getReefPoses(false);
    scheduler = CommandScheduler.getInstance();
    robotContainer = new RobotContainer();
  }

  @BeforeEach // this method will run before each test
  void setup() {
    resetCommandSchedule();
    setDSEnabled(true);
  }

  private void resetCommandSchedule() {
    scheduler.cancelAll();
    scheduler.enable();
    scheduler.getActiveButtonLoop().clear();
    scheduler.clearComposedCommands();
    scheduler.unregisterAllSubsystems();
  }

  public void setDSEnabled(boolean enabled) {
    DriverStationSim.setDsAttached(true);

    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    while (DriverStation.isEnabled() != enabled) {
      try {
        Thread.sleep(1);
      } catch (InterruptedException exception) {
        exception.printStackTrace();
      }
    }
  }

  /**
   * This test demonstrates how the default commands works using a single subsystem as an example.
   */
  @Test
  void defaultCommandWithSingleSubsystem() {
    Subsystem arm = RobotContainer.getArm();

    var defaultArmCommand = new TestCommand("Default Arm Command", arm);
    arm.setDefaultCommand(defaultArmCommand);

    // -- First Run of Scheduler (imagine this is a single periodic loop)
    scheduler.run();
    // The first call to schedule() executes commands that have been scheduled first
    // and then it schedules the default command. Scheduling a command doesn't execute it
    // which is why the executeCount is 0.
    assertEquals(0, defaultArmCommand.executeCount);
    // Also make sure that the default command was scheduled.
    assertTrue(defaultArmCommand.isScheduled());

    // -- Second Run of Scheduler
    scheduler.run();
    // This time the default command is actually executed, we verify that with the executeCount.
    assertEquals(1, defaultArmCommand.executeCount);

    // -- Third Run of Scheduler
    scheduler.run();
    // This time we make sure that our default command is still running.
    assertEquals(2, defaultArmCommand.executeCount);
    assertSame(defaultArmCommand, arm.getCurrentCommand());

    // -- Fourth Run of Scheduler
    // This time we will create a new command to move the arm and schedule it.
    // This should stop the default command from running so executeCount should not increase.
    var moveArmCommand = new TestCommand("Move Arm Command", arm);
    moveArmCommand.schedule();
    scheduler.run();
    assertEquals(1, moveArmCommand.executeCount);
    // The default command executeCount shouldn't change.
    assertEquals(2, defaultArmCommand.executeCount);
    // Also confirm that the default command is no longer scheduled.
    assertFalse(defaultArmCommand.isScheduled());

    // -- Fifth Run of Scheduler
    // Now, we let the moveArmCommand to continue to run, so the executeCount for it will
    // increase but the default command should not run.
    scheduler.run();
    assertEquals(2, moveArmCommand.executeCount);
    // The default command executeCount shouldn't change.
    assertEquals(2, defaultArmCommand.executeCount);
    // Also confirm that the default command is no longer scheduled.
    assertFalse(defaultArmCommand.isScheduled());

    // -- Sixth Run of Scheduler
    // Now, we will mark the moveArmCommand as finished, but it will get
    // executed one last time and get unscheduled and the default command should get scheduled.
    moveArmCommand.shouldFinish = true;
    scheduler.run();
    assertFalse(moveArmCommand.isScheduled());
    assertTrue(defaultArmCommand.isScheduled());
    assertEquals(3, moveArmCommand.executeCount);
    assertEquals(2, defaultArmCommand.executeCount);

    // -- Seventh Run of Scheduler
    // One last time to confirm that only default command is executed and not the moveArmCommand.
    scheduler.run();
    assertEquals(3, moveArmCommand.executeCount);
    assertEquals(3, defaultArmCommand.executeCount);
  }

  /**
   * This test validates that default commands should stop when a scoring command is scheduled and
   * run, which in real environment is when the joystick button is pressed.
   */
  @Test
  void defaultCommandsShouldStopWhenScoring() {
    // Register the default commands.
    robotContainer.initDefaultCommands();
    scheduler.run();
    // Make sure the default commands are now scheduled
    assertDefaultCommandsScheduled(true);
    // ... and running
    assertDefaultCommandsRunning();

    // Now schedule a scoring command which doesn't run immediately.
    var scoringCommand = ScoringFactory.score(Level.L4);
    scoringCommand.schedule();
    assertTrue(scoringCommand.isScheduled());

    // It takes two scheduler.run() calls to actually have the default commands
    // unscheduled.
    scheduler.run();
    scheduler.run();
    // The default commands should not be scheduled anymore.
    assertDefaultCommandsScheduled(false);

    // Cancel the scoring command, which is equivalent to letting go of the joystick button.
    scoringCommand.cancel();
    // Scoring command should not be scheduled anymore.
    assertFalse(scoringCommand.isScheduled());
    scheduler.run();

    scheduler.run();
    scheduler.run();
    assertDefaultCommandsRunning();
    assertDefaultCommandsScheduled(true);
  }

  private void assertDefaultCommandsScheduled(boolean expectedScheduledState) {
    assertEquals(
        expectedScheduledState, RobotContainer.getElevator().getDefaultCommand().isScheduled());
    assertEquals(expectedScheduledState, RobotContainer.getArm().getDefaultCommand().isScheduled());
    assertEquals(
        expectedScheduledState, RobotContainer.getEndEffector().getDefaultCommand().isScheduled());
  }

  private static void assertDefaultCommandsRunning() {
    System.out.println("Elevator: " + RobotContainer.getElevator().getCurrentCommand().getName());
    assertSame(
        RobotContainer.getElevator().getDefaultCommand(),
        RobotContainer.getElevator().getCurrentCommand());
    assertSame(
        RobotContainer.getArm().getDefaultCommand(), RobotContainer.getArm().getCurrentCommand());
    assertSame(
        RobotContainer.getEndEffector().getDefaultCommand(),
        RobotContainer.getEndEffector().getCurrentCommand());
  }

  private static class TestCommand extends Command {
    private int executeCount = 0;
    private boolean shouldFinish = false;

    TestCommand(String name, Subsystem... requirements) {
      super();
      setName(name);
      addRequirements(requirements);
    }

    @Override
    public void execute() {
      executeCount++;
      System.out.println(getName() + " executed: " + executeCount);
    }

    @Override
    public boolean isFinished() {
      return shouldFinish;
    }
  }
}
