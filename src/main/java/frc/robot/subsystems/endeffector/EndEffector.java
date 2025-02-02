package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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
        
        Logger.recordOutput("EndEffector/Current", inputs.statorCurrentAmps);
        Logger.recordOutput("EndEffector/MotorRunning", isRunning);
    }

    public Command runForward(double voltage) {
        isRunning = true;
        return runEnd(() -> io.setVoltage(voltage), () -> {
            io.stopMotor();
            isRunning = false;
        });
    }

    public Command runReverse(double voltage) {
        isRunning = true;
        return runEnd(() -> io.setVoltage(-voltage), () -> {
            io.stopMotor();
            isRunning = false;
        });
    }

    public Command stop() {
        isRunning = false;
        return runOnce(io::stopMotor);
    }

    public boolean isRunning() {
        return isRunning;
    }
}