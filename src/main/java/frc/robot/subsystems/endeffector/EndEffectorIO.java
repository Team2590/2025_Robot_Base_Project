package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs implements LoggableInputs {
        public double statorCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    /** Set motor percent output */
    public default void setMotor(double percentOutput) {}

    /** Stops the motor */
    public default void stopMotor() {}

    public static Runnable toggleMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toggleMotor'");
    }
}