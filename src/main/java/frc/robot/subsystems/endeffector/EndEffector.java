package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private final TalonFX motor = new TalonFX(2);
    private final XboxController controller = new XboxController(2);
    private boolean isRunning = false;

    @Override
    public void periodic() {
        //curent logging
        double current = motor.getStatorCurrent().getValueAsDouble();
        Logger.recordOutput("EndEffector/Current", current);
        
        //ported to A on xbox
        if (controller.getAButtonPressed()) {
            toggleMotor();
        }
        
//testing purposes, log motor even running
        Logger.recordOutput("EndEffector/MotorRunning", isRunning);
    }

    public void runMotor() {
        motor.set(0.5); // 50% power
        isRunning = true;
    }

    public void stopMotor() {
        motor.set(0);
        isRunning = false;
    }

    public void toggleMotor() {
        if (isRunning) {
            stopMotor();
        } else {
            runMotor();
        }
    }
}