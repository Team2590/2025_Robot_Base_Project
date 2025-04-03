// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.climb.Climb;
// import frc.robot.util.NemesisMathUtil;
// import java.util.Set;

// public class ClimbCommand extends Command {

//   // private Trigger limitSwitch = new Trigger(() ->
//   // RobotContainer.getClimb().getLimitSwitchValue());
//   private Climb climb;
//   private boolean pressed = false;
//   // private Command deployMech =
//   //     Commands.sequence(
//   //         new MoveFromHandoffCommand(
//   //             Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
//   //             .33,
//   //             Constants.ArmConstantsLeonidas.ARM_SET_STOW),
//   //         ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION));

//   public ClimbCommand() {
//     setName("Climb Command");
//     addRequirements(
//         Set.of(
//             RobotContainer.getArm(),
//             RobotContainer.getElevator(),
//             RobotContainer.getClimb(),
//             RobotContainer.getIntake()));
//   }

//   public void initialize() {
//     climb = RobotContainer.getClimb();
//
// RobotContainer.getElevator().getIO().setPosition(Constants.ArmConstantsLeonidas.ARM_SET_STOW);
//     RobotContainer.getIntake()
//         .getArmIO()
//         .setPosition(Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS);
//     RobotContainer.getArm().getIO().setPosition(Constants.ArmConstantsLeonidas.ARM_SET_STOW);
//   }

//   @Override
//   public void execute() {
//     // Set subsystems in correct positions
//     if (climb.getLimitSwitchValue()) pressed = true;

//     if (pressed) {
//       if (climb.getRotationCount() < Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION) {
//         climb.getIO().setVoltage(Constants.ClimbConstantsLeonidas.CLIMB_VOLTAGE);
//       } else {
//         climb.getIO().stop();
//       }
//     } else {
//       if (climb.getRotationCount() < Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION) {
//         climb.getIO().setVoltage(Constants.ClimbConstantsLeonidas.CLIMB_VOLTAGE);
//       } else {
//         climb.getIO().stop();
//       }
//     }
//   }

//   @Override
//   public boolean isFinished() {
//     return NemesisMathUtil.isApprox(
//         RobotContainer.getClimb().getRotationCount(),
//         1,
//         Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION);
//   }
// }
