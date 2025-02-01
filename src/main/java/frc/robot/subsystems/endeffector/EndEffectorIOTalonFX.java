package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    private final TalonFX motor;

    public EndEffectorIOTalonFX() {
        motor = new TalonFX(2);  // Motor ID 2
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setMotor(double percentOutput) {
        motor.set(percentOutput);
    }

    @Override
    public void stopMotor() {
        motor.set(0);
    }
}