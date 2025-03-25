package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;

public class AbsoluteEncoderEmulator {
    private double zeroOffset;
    private TalonFX talon;
    private double ratio;

    /**
     * @param talon - the motor that this emulator is being used on
     */
    public AbsoluteEncoderEmulator(TalonFX talon) {
        this(talon, 0, 1);
    }

    /**
     * @param talon - the motor that this emulator is being used on
     * @param zeroOffset - should be a negative number
     */
    public AbsoluteEncoderEmulator(TalonFX talon, double zeroOffset) {
        this(talon, zeroOffset, 1);        
    }

    /**
     * @param talon - the motor that this emulator is being used on
     * @param zeroOffset - should be a negative number
     * @param ratio - the ratio of motor rotations to mechanism rotations
     */
    public AbsoluteEncoderEmulator(TalonFX talon, double zeroOffset, double ratio) {
        this.zeroOffset = zeroOffset;
        this.talon = talon;
        this.ratio = ratio;
    }

    public double getVelocity() {
        return talon.getVelocity().getValueAsDouble();
    }
    
    public double getPosition() {
        return talon.getPosition().getValueAsDouble() - zeroOffset;
    }
    
    public double getAbsolutePosition() {
        return this.getPosition() % 1;
    }

    public double absolutePositionToRelative(double position) {
        return Math.floor(this.getPosition()) + position;
    }

    public double mechanismRotationsToCounts(double rotations) {
        return rotations * ratio;
    }

    public double getMechanismPosition() {
        return this.getPosition() / ratio;
    }

    public double getAbsoluteMechanismPosition() {
        return this.getMechanismPosition() % 1;
    }
}
