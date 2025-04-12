package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ScoringFactory.Level;

public class JoystickFactory {
    public static Command intakeCoralGround() {
        return Commands.defer(() -> {
            if (RobotContainer.getControllerApp().getTarget().scoringLevel() == Level.L1) {
                return GamePieceFactory.intakeCoralNoHandoff().andThen(IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.L1_POS));
            } else {
                return GamePieceFactory.intakeCoralGroundAndHandoff();
            }
        }, GamePieceFactory.intakeAlgaeGround().getRequirements());
    }

    public static Command intakeAlgaeReef() {
        return GamePieceFactory.GrabAlgaeL2();
    }

    public static Command ejectIntake() {
        return IntakeFactory.runIntake(() -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED);
    }

    public static Command ejectEndeffector() {
        return EndEffectorFactory.runEndEffectorVoltage(Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE);
    }
}
