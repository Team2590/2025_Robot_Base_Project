package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();
    private boolean isRunning = false;

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
        Logger.recordOutput("EndEffector/MotorRunning", isRunning);
    }

    public void runMotor() {
        io.setMotor(0.5);
        isRunning = true;
    }

    public void stopMotor() {
        io.stopMotor();
        isRunning = false;
    }

    public void toggleMotor() {
        if (isRunning) {
            stopMotor();
        } else {
            runMotor();
        }
    }

    public double getCurrentAmps() {
        return inputs.statorCurrentAmps;
    }
}