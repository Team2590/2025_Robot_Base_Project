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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  //   private final Drive drive;
  //   private final Intake intake;
  private final Arm arm;
  private LoggedTunableNumber armSetpointTest = new LoggedTunableNumber("Arm/ArmSetpointTest", 0);
  //   private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(2);
  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    arm = new Arm(new ArmIOTalonFX());

    // switch (Constants.currentMode) {
    //   case KRONOS:
    //     // Real robot, instantiate hardware IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIOPigeon2(),
    //             new ModuleIOTalonFX(TunerConstantsKronos.FrontLeft),
    //             new ModuleIOTalonFX(TunerConstantsKronos.FrontRight),
    //             new ModuleIOTalonFX(TunerConstantsKronos.BackLeft),
    //             new ModuleIOTalonFX(TunerConstantsKronos.BackRight));
    //     vision =
    //         new Vision(
    //             drive::addVisionMeasurement,
    //             new VisionIOPhotonVision(
    //                 List.of(
    //                     new CameraConfig(camera0Name, robotToCamera0),
    //                     new CameraConfig(camera1Name, robotToCamera1),
    //                     new CameraConfig(camera2Name, robotToCamera2),
    //                     new CameraConfig(camera3Name, robotToCamera3))));
    //     intake = null;
    //     break;

    //   case SIM:
    //     // Sim robot, instantiate physics sim IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIOSim(TunerConstants.FrontLeft),
    //             new ModuleIOSim(TunerConstants.FrontRight),
    //             new ModuleIOSim(TunerConstants.BackLeft),
    //             new ModuleIOSim(TunerConstants.BackRight));
    //     vision =
    //         new Vision(
    //             drive::addVisionMeasurement,
    //             new VisionIOPhotonVisionSim(
    //                 List.of(
    //                     new CameraConfig(camera0Name, robotToCamera0),
    //                     new CameraConfig(camera1Name, robotToCamera1),
    //                     new CameraConfig(camera2Name, robotToCamera2),
    //                     new CameraConfig(camera3Name, robotToCamera3)),
    //                 drive::getPose));
    //     intake = new Intake(new IntakeIOSim(DCMotor.getFalcon500(1), 4, .1));
    //     break;

    //   default:
    //     // Replayed robot, disable IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {});
    //     vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    //     intake = null;
    //     break;
    // }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -leftJoystick.getY(),
    //         () -> -leftJoystick.getX(),
    //         () -> -rightJoystick.getX()));

    // Lock to 0° when A button is held
    // controller.a().whileTrue(DriveCommands.driveToPose(new Pose2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.b().whileTrue(intake.runIntake(4));

    // Reset gyro to 0° when B button is pressed
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
    rightJoystick.button(1).whileTrue(arm.setPosition(Constants.ArmConstants.REEF_1_SETPOINT));
    leftJoystick.button(1).whileTrue(arm.setPosition(Constants.ArmConstants.REEF_2_3_SETPOINT));
    rightJoystick.button(2).whileTrue(arm.setPosition(Constants.ArmConstants.BARGE));
    leftJoystick
        .button(2)
        .whileTrue(arm.setPosition(Constants.ArmConstants.GROUND_INTAKE_SETPOINT));
    rightJoystick
        .button(3)
        .whileTrue(arm.setPosition(Constants.ArmConstants.CORAL_STATION_INTAKE_SETPOINT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //   public Command getAutonomousCommand() {
  //     return autoChooser.get();
  //   }
}
