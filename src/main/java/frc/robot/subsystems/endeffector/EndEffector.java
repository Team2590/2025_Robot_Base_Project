package frc.robot.subsystems.endeffector;

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

    public void runForward() {
        io.setMotor(0.5); 
        isRunning = true;
    }

    public void runReverse() {
        io.setMotor(-0.5);  
        isRunning = true;
    }

    public void stop() {
        io.stopMotor();
        isRunning = false;
    }

    public boolean isRunning() {
        return isRunning;
    }
}