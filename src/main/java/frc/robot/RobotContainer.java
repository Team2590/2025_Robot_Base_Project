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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstantsLarry;
import frc.robot.Constants.EndEffectorConstantsLeonidas;
import frc.robot.command_factories.GamePieceFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.generated.TunerConstantsWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
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
                        new CameraConfig(sourceCameraName, robotToSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam))));
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
                        new CameraConfig(sourceCameraName, robotToSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam))));
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
        vision = null;
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
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
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
                        new CameraConfig(sourceCameraName, robotToSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam))));
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
                        new CameraConfig(sourceCameraName, robotToSourceCam),
                        new CameraConfig(processorCameraName, robotToProcessorCam))));
        intake =
            new Intake(
                new IntakeIOTalonFX(60, "Takeover", 20, false, true, 1),
                new IntakeArmIOTalonFX(50, "Takeover", 20, true, true, 1));
        arm = new Arm(null);
        elevator = null;
        endEffector = null;
        break;
    }

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -leftJoystick.getY(),
    //         () -> -leftJoystick.getX(),
    //         () -> -rightJoystick.getX()));

    // Default drive command using new factory method, replacement for above ^^.
    // drive.setDefaultCommand(DriveFactory.joystickDrive());

    // Lock to 0° when A button is held
    // controller.a().whileTrue(DriveCommands.driveToPose(new Pose2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // // controller.b().whileTrue(intake.runIntake(4));

    // rightJoystick.button(1).whileTrue(endEffector.runEndEffectorOuttake());
    // leftJoystick.button(1).whileTrue(endEffector.runEndEffector());
    // rightJoystick.button(3).onTrue(elevator.setPosition(50));
    // rightJoystick
    //     .button(2)
    //     .onTrue(
    //
    // elevator.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS));

    //////////////////////////////////////////////////////
    /// Examples of using commands from command factories.
    //////////////////////////////////////////////////////
    ///
    ///
    // example of how we might change some of the above to using the elevator factory
    // leftJoystick.pov(0).onTrue(ElevatorFactory.setPosition(5));
    // leftJoystick.pov(90).onTrue(ElevatorFactory.setPosition(17));
    // leftJoystick.pov(180).onTrue(ElevatorFactory.setPosition(30));
    // leftJoystick.pov(270).onTrue(ElevatorFactory.setPosition(48));
    // leftJoystick.button(5).onTrue(ElevatorFactory.resetRotationCount());
    // leftJoystick.button(4).onTrue(ElevatorFactory.setPosition(0));
    // leftJoystick.button(1).whileTrue(endEffector.intake());

    // Example of going to a specific pose using a command
    // leftJoystick.butotn(2).onTrue(DriveFactory.driveToPose(new Pose2d()));

    // Example of intake commands using controller buttons and factory pattern
    // leftJoystick.button(1).whileTrue(IntakeFactory.runIntake(() -> 8));
    // leftJoystick.button(2).whileTrue(IntakeFactory.runIntake(() -> -8));
    // leftJoystick.button(3).onTrue(IntakeFactory.setIntakeCoralPosition());
    // leftJoystick.button(4).onTrue(IntakeFactory.setIntakeAlgaePosition());

    // Example of scoring commands using controller buttons
    // leftJoystick.button(6).onTrue(ScoringFactory.scoreHigh());
    // leftJoystick.button(7).onTrue(ScoringFactory.scoreMid());
    // leftJoystick.button(8).onTrue(ScoringFactory.stow());
    //////////////////////////////////////////////////////
    /// End of examples using command factories
    //////////////////////////////////////////////////////

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
    // rightJoystick
    //     .button(5)
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // rightJoystick.button(1).whileTrue(endEffector.runEndEffectorOuttake());
    // leftJoystick.button(1).whileTrue(endEffector.runEndEffector());
    // rightJoystick.button(3).onTrue(elevator.setPosition(50));
    // rightJoystick
    //     .button(2)
    //     .onTrue(
    //
    // elevator.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS));
    // rightJoystick.button(1).onTrue(elevator.setPosition());

    rightJoystick.button(1).onTrue(ScoringFactory.scoreL1());
    rightJoystick.button(2).onTrue(ScoringFactory.scoreL2());
    rightJoystick.button(3).onTrue(ScoringFactory.scoreL3());
    rightJoystick.button(4).onTrue(ScoringFactory.scoreL4());
    leftJoystick.button(1).onTrue(elevator.setPosition(5));
    rightJoystick.button(5).onTrue(GamePieceFactory.intakeCoralFeeder());
    controller.x().onTrue(elevator.resetRotationCountCommand());

    // leftJoystick.button(1).

    /**
     * the following button binds work on Larry:
     * leftJoystick.pov(0).onTrue(elevator.setPosition(5));
     * leftJoystick.pov(90).onTrue(elevator.setPosition(17));
     * leftJoystick.pov(180).onTrue(elevator.setPosition(30));
     * leftJoystick.pov(270).onTrue(elevator.setPosition(48));
     * leftJoystick.button(5).onTrue(elevator.resetRotationCount());
     * leftJoystick.button(4).onTrue(elevator.setPosition(0));
     * rightJoystick.button(1).whileTrue(endEffector.intake());
     * leftJoystick.button(1).whileTrue(endEffector.outtake());
     * leftJoystick.button(2).whileTrue(intake.runIntake(-2));
     * rightJoystick.button(2).onTrue(intake.setIntakeCoralPosition());
     * rightJoystick.button(2).whileTrue(intake.runIntake(8));
     * rightJoystick.button(5).onTrue(intake.resetArmRotationCount());
     * rightJoystick.button(2).onFalse(intake.setPosition(0));
     * rightJoystick.button(3).onTrue(intake.setIntakeAlgaePosition());
     * rightJoystick.button(3).whileTrue(intake.runIntake(-8));
     * rightJoystick.button(3).onFalse(intake.setPosition(0));
     * rightJoystick.button(4).whileTrue(intake.runIntake(4));
     */

    // rightJoystick.button(1).whileTrue(arm.setPosition(Constants.ArmConstants.REEF_1_SETPOINT));
    // leftJoystick.button(1).whileTrue(arm.setPosition(Constants.ArmConstants.REEF_2_3_SETPOINT));
    // rightJoystick.button(2).whileTrue(arm.setPosition(Constants.ArmConstants.BARGE));
    // leftJoystick
    //     .button(2)
    //     .whileTrue(arm.setPosition(Constants.ArmConstants.GROUND_INTAKE_SETPOINT));
    // rightJoystick
    //     .button(3)
    //     .whileTrue(arm.setPosition(Constants.ArmConstants.CORAL_STATION_INTAKE_SETPOINT));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Add joystick accessors if missing
  public static CommandXboxController getDriverController() {
    return controller;
  }

  public static CommandXboxController getOperatorController() {
    return controller;
  }
}
