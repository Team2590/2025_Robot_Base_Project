package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicReference;

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

  public Command setOff() {
    LEDPattern.kOff.applyTo(buffer);
    return runOnce(() -> led.setData(buffer));
  }

  public void off() {
    LEDPattern.kOff.applyTo(buffer);
    led.setData(buffer);
  }

  // public Command rainbow() {
  //     Distance LedSpacing = Meters.of(1 / 120);
  //     LEDPattern rainbow = LEDPattern.rainbow(255,
  // 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LedSpacing);
  //     rainbow.applyTo(buffer);
  //     return runOnce(() -> led.setData(buffer));
  // }

  public Command setRainbowFlow() {
    Distance LedSpacing = Meters.of(1 / 120);
    LEDPattern rainbow =
        LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LedSpacing);
    rainbow.applyTo(buffer);
    return runOnce(() -> led.setData(buffer))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command setNemesisFlow() {
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kWhite);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);

    pattern.applyTo(buffer);
    led.setData(buffer);

    return Commands.none();
  }

  public Command setColor(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(buffer);

    return runOnce(() -> led.setData(buffer))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command setBlinking(Color color, double seconds, double interval) {
    double startTime = Timer.getFPGATimestamp();
    AtomicReference<Double> lastTimeStamp = new AtomicReference<Double>(startTime);

    return run(() -> {
          double currentTime = Timer.getFPGATimestamp();
          if (lastTimeStamp.get() + interval > currentTime) {
            off();
          } else {
            LEDPattern.solid(color).applyTo(buffer);
            led.setData(buffer);
          }

          lastTimeStamp.set(Timer.getFPGATimestamp());
        })
        .until(() -> Timer.getFPGATimestamp() >= startTime + seconds)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
}
