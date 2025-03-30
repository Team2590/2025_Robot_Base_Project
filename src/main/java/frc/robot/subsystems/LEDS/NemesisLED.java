package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.LEDFactory;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class NemesisLED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView viewStart;
  private final AddressableLEDBufferView viewEnd;

  private LEDPattern patternStart;
  private LEDPattern patternEnd;

  public NemesisLED(int port, int length, int halfWay) {
    // Initialize the LED hardware
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    viewStart = buffer.createView(0, halfWay);
    viewEnd = buffer.createView(halfWay + 1, length - 1);
    led.setLength(length);
    led.start();

    patternStart = LEDPattern.kOff;
    patternEnd = LEDPattern.kOff;

    setDefaultCommand(runOnce(() -> LEDPattern.kOff.applyTo(buffer)));
  }

  @Override
  public void periodic() {
    patternStart.applyTo(viewStart);
    patternEnd.applyTo(viewEnd);

    led.setData(buffer);

    if (RobotState.endEffectorHasGamePiece() && Vision.seesReefTag()){
      LEDFactory.readyDriveToPose();
    }
    else{
      LEDFactory.solid();
    }
  }

  public Command setNemesisFlow(Color color1, Color color2) {
    var scrollSpeed = MetersPerSecond.of(3);
    var stripLength = Meters.of(0.05);
    var scrollingPattern = LEDPattern.gradient(GradientType.kContinuous, color1, color2);
    return runOnce(
        () -> {
          patternStart = scrollingPattern.scrollAtAbsoluteSpeed(scrollSpeed, stripLength);
          patternEnd = scrollingPattern.scrollAtAbsoluteSpeed(scrollSpeed, stripLength);
        });
  }

  public Command setColor(Color color1, Color color2) {
    return runOnce(
        () -> {
          patternStart = LEDPattern.solid(color1);
          patternEnd = LEDPattern.solid(color2);
        });
  }

  public void setColorVoid(Color color1, Color color2) {
    patternStart = LEDPattern.solid(color1);
    patternEnd = LEDPattern.solid(color2);
  }

  public Command setBlink(Color color1, Color color2, double onTime) {
    return runOnce(
        () -> {
          patternStart = LEDPattern.solid(color1).blink(Seconds.of(onTime));
          patternEnd = LEDPattern.solid(color2).blink(Seconds.of(onTime));
        });
  }

  public Command setAuraRizz(Color color1, Color color2) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    var stripLength = Meters.of(0.05);
    var scrollSpeed = MetersPerSecond.of(3);
    var base = LEDPattern.gradient(GradientType.kContinuous, color1, color2);
    var mask = LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(scrollSpeed, stripLength);

    return runOnce(
        () -> {
          patternStart = base.mask(mask);
          patternEnd = base.mask(mask);
        });
  }

  private double progressMaskCount = 0;

  public Command setProgressMask(Color color1, Color color2) {
    var progressMask =
        LEDPattern.progressMaskLayer(
            () -> {
              progressMaskCount += 0.01;
              return Math.abs(Math.cos(progressMaskCount));
            });
    var pattern1 = LEDPattern.solid(color1);
    var pattern2 = LEDPattern.solid(color2);

    return runOnce(
        () -> {
          patternStart = pattern1.mask(progressMask);
          patternEnd = pattern2.mask(progressMask);
        });
  }
}
