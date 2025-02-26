// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstantsLarry;
import frc.robot.Constants.EndEffectorConstantsLeonidas;
import frc.robot.command_factories.DriveFactory;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.command_factories.EndEffectorFactory;
import frc.robot.command_factories.GamePieceFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.generated.TunerConstantsWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeArmIOSim;
import frc.robot.subsystems.intake.IntakeArmIOTalonFX;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVision.CameraConfig;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  @Getter private static Drive drive;
  @Getter private static Vision vision;
  @Getter private static Arm arm;
  @Getter private static Elevator elevator;
  @Getter private static Intake intake;
  @Getter private static EndEffector endEffector;
  @Getter private static Climb climb;
  private final ControllerOrchestrator controllerApp = new ControllerOrchestrator();

  // private final Intake intake;
  public static final TunerConstantsWrapper constantsWrapper = new TunerConstantsWrapper();

  // Controller
  @Getter private static CommandXboxController controller = new CommandXboxController(2);

  @Getter private static CommandJoystick leftJoystick = new CommandJoystick(0);

  @Getter private static CommandJoystick rightJoystick = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case KRONOS:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(constantsWrapper.FrontLeft, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.FrontRight, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.BackLeft, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.BackRight, constantsWrapper),
                constantsWrapper);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    List.of(
                        new CameraConfig(upperSourceCameraName, robotToUpperSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam),
                        new CameraConfig(reefCameraName, robotToReefCam))));
        intake =
            new Intake(
                new IntakeIOTalonFX(60, "Takeover", 20, false, true, 1),
                new IntakeArmIOTalonFX(50, "Takeover", 20, true, true, 1));
        arm =
            new Arm(
                new ArmIOTalonFX(
                    Constants.ArmConstantsKronos.ARM_CAN_ID,
                    Constants.ArmConstantsKronos.CANBUS,
                    Constants.ArmConstantsKronos.CURRENT_LIMIT,
                    Constants.ArmConstantsKronos.INVERT,
                    Constants.ArmConstantsKronos.BRAKE,
                    Constants.ArmConstantsKronos.REDUCTION,
                    Constants.ArmConstantsKronos.ARM_CANCODER_ID,
                    Constants.ArmConstantsKronos.MAG_OFFSET,
                    Constants.ArmConstantsKronos.SENSOR_REDUCTION));
        elevator = null;
        endEffector = null;
        climb = null;
        break;
      case LARRY:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(constantsWrapper.FrontLeft, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.FrontRight, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.BackLeft, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.BackRight, constantsWrapper),
                constantsWrapper);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    List.of(
                        new CameraConfig(upperSourceCameraName, robotToUpperSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam),
                        new CameraConfig(reefCameraName, robotToReefCam))));
        intake =
            new Intake(
                new IntakeIOTalonFX(60, "Takeover", 20, false, true, 1),
                new IntakeArmIOTalonFX(50, "Takeover", 20, true, true, 1));
        arm =
            new Arm(
                new ArmIOTalonFX(
                    Constants.ArmConstantsLarry.ARM_CAN_ID,
                    Constants.ArmConstantsLarry.CANBUS,
                    Constants.ArmConstantsLarry.CURRENT_LIMIT,
                    Constants.ArmConstantsLarry.INVERT,
                    Constants.ArmConstantsLarry.BRAKE,
                    Constants.ArmConstantsLarry.REDUCTION,
                    Constants.ArmConstantsLarry.ARM_CANCODER_ID,
                    Constants.ArmConstantsLarry.MAG_OFFSET,
                    Constants.ArmConstantsLarry.SENSOR_REDUCTION));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    ElevatorConstantsLarry.canID,
                    ElevatorConstantsLarry.canBus,
                    ElevatorConstantsLarry.currentLimitAmps,
                    ElevatorConstantsLarry.invert,
                    ElevatorConstantsLarry.brake,
                    ElevatorConstantsLarry.reduction));
        endEffector =
            new EndEffector(
                new EndEffectorIOTalonFX(0, "Takeover", 120, false, true, angularStdDevBaseline));
        climb = null;
        break;
      case Leonidas:
        drive =
            new Drive(
                new GyroIOPigeon2() {},
                new ModuleIOTalonFX(constantsWrapper.FrontLeft, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.FrontRight, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.BackLeft, constantsWrapper),
                new ModuleIOTalonFX(constantsWrapper.BackRight, constantsWrapper),
                constantsWrapper);
        arm =
            new Arm(
                new ArmIOTalonFX(
                    Constants.ArmConstantsLeonidas.canID,
                    Constants.ArmConstantsLeonidas.canBus,
                    Constants.ArmConstantsLeonidas.currentLimitAmps,
                    Constants.ArmConstantsLeonidas.invert,
                    Constants.ArmConstantsLeonidas.brake,
                    Constants.ArmConstantsLeonidas.reduction,
                    Constants.ArmConstantsLeonidas.cancoderID,
                    Constants.ArmConstantsLeonidas.magOffset,
                    Constants.ArmConstantsLeonidas.sensorReduction));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    Constants.ElevatorConstantsLeonidas.canID,
                    Constants.ElevatorConstantsLeonidas.canBus,
                    Constants.ElevatorConstantsLeonidas.currentLimitAmps,
                    Constants.ElevatorConstantsLeonidas.invert,
                    Constants.ElevatorConstantsLeonidas.brake,
                    Constants.ElevatorConstantsLeonidas.reduction));
        elevator.resetRotationCount();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    List.of(
                        new CameraConfig(upperSourceCameraName, robotToUpperSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam),
                        new CameraConfig(reefCameraName, robotToReefCam))));
        intake =
            new Intake(
                new IntakeIOTalonFX(
                    Constants.IntakeConstantsLeonidas.canID,
                    Constants.IntakeConstantsLeonidas.canBus,
                    Constants.IntakeConstantsLeonidas.currentLimitAmps,
                    Constants.IntakeConstantsLeonidas.invert,
                    Constants.IntakeConstantsLeonidas.brake,
                    Constants.IntakeConstantsLeonidas.reduction),
                new IntakeArmIOTalonFX(
                    Constants.IntakeArmConstantsLeonidas.canID,
                    Constants.IntakeArmConstantsLeonidas.canBus,
                    Constants.IntakeArmConstantsLeonidas.currentLimitAmps,
                    Constants.IntakeArmConstantsLeonidas.invert,
                    Constants.IntakeArmConstantsLeonidas.brake,
                    Constants.IntakeArmConstantsLeonidas.reduction));
        intake.resetArmRotationCount();
        endEffector =
            new EndEffector(
                new EndEffectorIOTalonFX(
                    Constants.EndEffectorConstantsLeonidas.canID,
                    Constants.EndEffectorConstantsLeonidas.canBus,
                    Constants.EndEffectorConstantsLeonidas.currentLimitAmps,
                    Constants.EndEffectorConstantsLeonidas.invert,
                    Constants.EndEffectorConstantsLeonidas.brake,
                    Constants.EndEffectorConstantsLeonidas.reduction));
        climb =
            new Climb(
                new ClimbIOTalonFX(
                    Constants.ClimbConstantsLeonidas.canID,
                    Constants.ClimbConstantsLeonidas.canBus,
                    Constants.ClimbConstantsLeonidas.currentLimitAmps,
                    Constants.ClimbConstantsLeonidas.invert,
                    Constants.ClimbConstantsLeonidas.brake,
                    Constants.ClimbConstantsLeonidas.reduction));
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(constantsWrapper.FrontLeft, constantsWrapper),
                new ModuleIOSim(constantsWrapper.FrontRight, constantsWrapper),
                new ModuleIOSim(constantsWrapper.BackLeft, constantsWrapper),
                new ModuleIOSim(constantsWrapper.BackRight, constantsWrapper),
                constantsWrapper);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    List.of(
                        new CameraConfig(upperSourceCameraName, robotToUpperSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam),
                        new CameraConfig(reefCameraName, robotToReefCam))));
        intake =
            new Intake(
                new IntakeIOSim(DCMotor.getFalcon500(1), 4, .1),
                new IntakeArmIOSim(DCMotor.getFalcon500(1), 4, .1));
        arm = new Arm(new ArmIOSim(DCMotor.getFalcon500(1), 1, 1, 1, 1, 1, true, 1));
        elevator =
            new Elevator(new ElevatorIOSim(DCMotor.getFalcon500(1), 1, 1, 1, 1, 10, false, 1));
        endEffector =
            new EndEffector(
                new EndEffectorIOSim(
                    DCMotor.getFalcon500(1), EndEffectorConstantsLeonidas.reduction, 1));
        endEffector =
            new EndEffector(
                new EndEffectorIOSim(
                    DCMotor.getFalcon500(1), EndEffectorConstantsLeonidas.reduction, 1));
        climb = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                constantsWrapper);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    List.of(
                        new CameraConfig(upperSourceCameraName, robotToUpperSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam),
                        new CameraConfig(reefCameraName, robotToReefCam))));
        intake =
            new Intake(
                new IntakeIOTalonFX(60, "Takeover", 20, false, true, 1),
                new IntakeArmIOTalonFX(50, "Takeover", 20, true, true, 1));
        arm = new Arm(null);
        elevator = null;
        endEffector = null;
        climb = null;
        break;
    }
    RobotState.initialize(arm, drive, elevator, endEffector, intake, vision);

    // setup Named Commands:
    registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Elevator FF Characterization",
        new FeedForwardCharacterization(
            elevator, elevator::setVoltage, elevator::getCharacterizationVelocity));
    autoChooser.addOption(
        "Arm FF Characterization",
        new FeedForwardCharacterization(arm, arm::setVoltage, arm::getCharacterizationVelocity));
    autoChooser.addOption(
        "Intake FF Characterization",
        new FeedForwardCharacterization(
            intake, intake::setVoltage, intake::getCharacterizationVelocity));
  }

  /**
   * Configure buttons.
   *
   * <p>Called from {@link Robot#robotInit()}. Avoid calling from RobotContainer constructor as it
   * uses Subsystems factories which in turn use RobotContainer which is not fully initialized yet
   * since constructor is not fully executed, causing NullPointerExceptions at startup.
   */
  public void configureButtons() {
    // Configure the button bindings
    if (Constants.currentMode == Constants.Mode.SIM) {
      configureButtonBindingsSimulation();
    } else {
      configureButtonBindings();
    }
  }

  private void configureButtonBindingsSimulation() {

    // Add elevator control bindings
    leftJoystick
        .button(4)
        .onTrue(
            elevator.setPosition(
                Constants.ElevatorConstantsLeonidas
                    .ELEVATOR_OPERATIONAL_MIN_POS)); // Move to home position
    leftJoystick
        .button(5)
        .onTrue(
            elevator.setPosition(
                Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS)); // Just safe

    // Add arm control bindings
    leftJoystick
        .button(6)
        .onTrue(
            arm.setPosition(
                Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS)); // Min position
    leftJoystick
        .button(7)
        .onTrue(
            arm.setPosition(
                Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS)); // Max position

    leftJoystick.button(8).onTrue(ScoringFactory.score(Level.L3));
    leftJoystick.button(9).onTrue(ScoringFactory.scoreProcessor());

    // TODO(asim): These are only mapped in SIM, need to figure out how to map them in real robot
    leftJoystick.button(10).whileTrue(controllerApp.bindDriveToTargetCommand(drive));
    leftJoystick.button(11).whileTrue(controllerApp.bindScoringCommand(elevator, arm));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default drive command using new factory method, replacement for above ^^.
    drive.setDefaultCommand(DriveFactory.joystickDrive());

    // reset button binds
    rightJoystick
        .button(5)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    leftJoystick.button(5).onTrue(Commands.runOnce(() -> elevator.resetRotationCount(), elevator));

    // climb button binds
    /**
     * TODO - need climb factory methods for running till end and deploying backpack
     * rightJoystick.button(4) -> backpack activation down leftJoystick.button(4) or button 3 -> run
     * full climb
     */

    // intake button binds
    rightJoystick
        .trigger()
        .and(rightJoystick.button(3).negate())
        .and(rightJoystick.button(2).negate())
        .whileTrue(GamePieceFactory.intakeAlgaeGround());
    rightJoystick
        .button(2)
        .and(rightJoystick.trigger())
        .whileTrue(GamePieceFactory.intakeCoralGround());

    rightJoystick
        .button(3)
        .and(rightJoystick.trigger())
        .whileTrue(GamePieceFactory.intakeCoralFeeder());
    
    

    // scoring button binds
    // TODO- controller app activation button:
    // rightJoystick.button(3).and(leftJoystick.trigger()).whileTrue(<controller app function>);
    // rightJoystick.button(3).onTrue(ScoringFactory.scoreL2());
    /**
     * For tuning purposes: rightJoystick .button(3) .and(leftJoystick.trigger()) .whileTrue( new
     * ParallelCommandGroup( arm.setPositionLoggedTunableNumber(),
     * elevator.setPositionLoggedTunableNumber())); controller.button(7).whileTrue( new
     * ParallelCommandGroup( arm.setPositionLoggedTunableNumber(),
     * elevator.setPositionLoggedTunableNumber()));
     */
    // manual backup button binds

    // controller
    //     .button(7)
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             arm.setPositionLoggedTunableNumber(),
    // elevator.setPositionLoggedTunableNumber()));

    rightJoystick
        .button(3)
        .and(leftJoystick.button(2))
        .whileTrue(EndEffectorFactory.runEndEffectorOuttake());
    rightJoystick.povUp().and(leftJoystick.button(4)).whileTrue(ElevatorFactory.manualUp());
    rightJoystick.povDown().and(leftJoystick.button(4)).whileTrue(ElevatorFactory.manualDown());

    // SCORING BUTTONS
    // rightJoystick
    //     .button(2)
    //     .and(rightJoystick.trigger())
    //     .whileTrue(GamePieceFactory.intakeCoralGround());
    // rightJoystick.button(2).and(rightJoystick.trigger()).whileTrue(ScoringFactory.scoreL1());

    leftJoystick
        .trigger()
        .and(rightJoystick.button(3).negate())
        .and(rightJoystick.button(2).negate())
        .whileTrue(ScoringFactory.scoreProcessor().finallyDo(() -> RobotState.setIntakeNoAlgae()));

    rightJoystick
        .button(2)
        .and(leftJoystick.trigger())
        .whileTrue(ScoringFactory.score(Level.L1).finallyDo(() -> RobotState.setIntakeNoCoral()));

    // controller.a().whileTrue(EndEffectorFactory.runEndEffectorOuttake());
    // controller.b().whileTrue(EndEffectorFactory.runEndEffector());
    // controller.rightBumper().onTrue(GamePieceFactory.intakeAlgaeGround());
    // controller.leftBumper().onTrue(GamePieceFactory.intakeCoralGround());
    controller.rightBumper().whileTrue(EndEffectorFactory.runEndEffectorOuttake());
    controller.leftBumper().whileTrue(EndEffectorFactory.runEndEffector());
    controller.a().whileTrue(ScoringFactory.score(Level.L1));
    controller.x().whileTrue(ScoringFactory.score(Level.L2));
    controller.b().whileTrue(ScoringFactory.score(Level.L3));
    controller.y().whileTrue(ScoringFactory.score(Level.L4));

    // Causing NullPointerException on startup in SIM
    // rightJoystick.button(11).whileTrue(ScoringFactory.deployMechanism());
    // rightJoystick.button(12).onTrue(ScoringFactory.prepClimb());
    // rightJoystick.button(16).whileTrue(ScoringFactory.climb());
  }

  /**
   * drive Use this to pass the autonomous command to the main
   * {@lifrc.robot.RobotContainer.configureButtonBindings(RobotContainer.java:509)nk Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static void registerNamedCommands() {

    // Prime Elevator and Arm position. Useful in Autos but could also be useful to trigger in a
    // zone
    // if we have a coral and the controller app telling us where to score.
    NamedCommands.registerCommand("PrimeL4", ScoringFactory.primeForLevel(ScoringFactory.Level.L4));
    NamedCommands.registerCommand("PrimeL3", ScoringFactory.primeForLevel(ScoringFactory.Level.L3));
    NamedCommands.registerCommand("PrimeL2", ScoringFactory.primeForLevel(ScoringFactory.Level.L2));
    // NamedCommands.registerCommand("PrimeL1",
    // ScoringFactory.primeForLevel(ScoringFactory.Level.L1));
    // TODO: Prime for Source

    // Scoring Commands
    NamedCommands.registerCommand("ScoreL4", ScoringFactory.score(ScoringFactory.Level.L4));
    NamedCommands.registerCommand("ScoreL3", ScoringFactory.score(ScoringFactory.Level.L3));
    NamedCommands.registerCommand("ScoreL2", ScoringFactory.score(ScoringFactory.Level.L2));
    NamedCommands.registerCommand("ScoreL1", ScoringFactory.score(ScoringFactory.Level.L1));

    // Does this need priming?
    NamedCommands.registerCommand("ScoreProcessor", ScoringFactory.scoreProcessor());

    NamedCommands.registerCommand("Stow-Mechanism", ScoringFactory.stow());
    NamedCommands.registerCommand("PrimeSource", ScoringFactory.stow());

    // This uses a wait command but we can make this into a WaitUntil command that can wait
    // for a certain condition.
    NamedCommands.registerCommand(
        "WaitAndPrint", Commands.waitSeconds(5).andThen(Commands.print("Done waiting ...")));
  }

  public boolean inReef() {
    return Constants.locator.getZoneOfField(drive.getPose()).equals("reef");
  }

  // Add joystick accessors if missing
  public static CommandXboxController getDriverController() {
    return controller;
  }

  public static CommandXboxController getOperatorController() {
    return controller;
  }
}
