package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs implements LoggableInputs {
        public double statorCurrentAmps = 0.0;
        public double motorSpeed = 0.0;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void setMotor(double speed) {}

    public default void stopMotor() {}
}