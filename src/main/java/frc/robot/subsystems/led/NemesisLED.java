package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class NemesisLED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private LEDPattern basePattern;

  public NemesisLED(int port, int length) {
    // Initialize the LED hardware
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.start();

    basePattern = LEDPattern.kOff;

    setDefaultCommand(runOnce(() -> LEDPattern.kOff.applyTo(buffer)));
  }

  @Override
  public void periodic() {
    basePattern.applyTo(buffer);

    led.setData(buffer);
  }

  public Command setNemesisFlow() {
    var scrollSpeed = MetersPerSecond.of(3);
    var stripLength = Meters.of(0.05);
    var scrollingPattern = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kWhite);
    basePattern = scrollingPattern.scrollAtAbsoluteSpeed(scrollSpeed, stripLength);
    return Commands.none();
  }

  public Command setColor(Color color) {
    basePattern = LEDPattern.solid(color);
    return Commands.none();
  }

  public Command setBlink(Color color, double onTime) {
    basePattern = LEDPattern.solid(color).blink(Seconds.of(onTime));
    return Commands.none();
  }

  public Command setAuraRizz() {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    var stripLength = Meters.of(0.05);
    var scrollSpeed = MetersPerSecond.of(3);
    var base = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kWhite);
    var mask = LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(scrollSpeed, stripLength);

    basePattern = base.mask(mask);
    return Commands.none();
  }

  private double progressMaskCount = 0;

  public Command setProgressMask(Color color) {
    var progressMask =
        LEDPattern.progressMaskLayer(
            () -> {
              progressMaskCount += 0.01;
              return Math.abs(Math.cos(progressMaskCount));
            });
    var pattern = LEDPattern.solid(color);
    basePattern = pattern.mask(progressMask);
    return Commands.none();
  }
}
