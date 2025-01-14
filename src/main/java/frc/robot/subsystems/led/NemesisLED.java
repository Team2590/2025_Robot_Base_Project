package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

/**
 * @author Dhruv Shah
 */
public class NemesisLED extends SubsystemBase {
    private AddressableLED led; 
    private AddressableLEDBuffer buffer;

    public NemesisLED(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    public Command stop() {
        return runOnce(led::stop);
    }

    public Command start() {
        return runOnce(led::start);
    }

    // public Command rainbow() {
    //     Distance LedSpacing = Meters.of(1 / 120);
    //     LEDPattern rainbow = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LedSpacing);
    //     rainbow.applyTo(buffer);
    //     return runOnce(() -> led.setData(buffer));
    // }

    public Command setRainbowFlow() {
        Distance LedSpacing = Meters.of(1 / 120);
        LEDPattern rainbow = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LedSpacing);
        rainbow.applyTo(buffer);
        return run(() -> led.setData(buffer)).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command setNemesisFlow() {
        LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kBlue);
        LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));

        pattern.applyTo(buffer);
        return runOnce(() -> led.setData(buffer));
    }

    public Command setColor(Color color) {
        LEDPattern pattern = LEDPattern.solid(color);
        pattern.applyTo(buffer);

        return runOnce(() -> led.setData(buffer));
    }
    
    // public Command setBlinking(Color color, double seconds) {

    // }
}
