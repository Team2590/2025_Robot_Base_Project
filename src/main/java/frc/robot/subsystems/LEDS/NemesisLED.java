package frc.robot.subsystems.LEDS;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

/** Control REV Robotics Blinkin LED controller */
public class NemesisLED {
  public enum BlinkinPattern {
    /*
     * Fixed Palette Pattern
     */
    RAINBOW_RAINBOW_PALETTE(-0.99),
    RAINBOW_PARTY_PALETTE(-0.97),
    RAINBOW_OCEAN_PALETTE(-0.95),
    RAINBOW_LAVA_PALETTE(-0.93),
    RAINBOW_FOREST_PALETTE(-0.91),
    RAINBOW_WITH_GLITTER(-0.89),
    CONFETTI(-0.87),
    SHOT_RED(-0.85),
    SHOT_BLUE(-0.83),
    SHOT_WHITE(-0.81),
    SINELON_RAINBOW_PALETTE(-0.79),
    SINELON_PARTY_PALETTE(-0.77),
    SINELON_OCEAN_PALETTE(-0.75),
    SINELON_LAVA_PALETTE(-0.73),
    SINELON_FOREST_PALETTE(-0.71),
    BEATS_PER_MINUTE_RAINBOW_PALETTE(-0.69),
    BEATS_PER_MINUTE_PARTY_PALETTE(-0.67),
    BEATS_PER_MINUTE_OCEAN_PALETTE(-0.65),
    BEATS_PER_MINUTE_LAVA_PALETTE(-0.63),
    BEATS_PER_MINUTE_FOREST_PALETTE(-0.61),
    FIRE_MEDIUM(-0.59),
    FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW_PALETTE(-0.55),
    TWINKLES_PARTY_PALETTE(-0.53),
    TWINKLES_OCEAN_PALETTE(-0.51),
    TWINKLES_LAVA_PALETTE(-0.49),
    TWINKLES_FOREST_PALETTE(-0.47),
    COLOR_WAVES_RAINBOW_PALETTE(-0.45),
    COLOR_WAVES_PARTY_PALETTE(-0.43),
    COLOR_WAVES_OCEAN_PALETTE(-0.41),
    COLOR_WAVES_LAVA_PALETTE(-0.39),
    COLOR_WAVES_FOREST_PALETTE(-0.37),
    LARSON_SCANNER_RED(-0.35),
    LARSON_SCANNER_GRAY(-0.33),
    LIGHT_CHASE_RED(-0.31),
    LIGHT_CHASE_BLUE(-0.29),
    LIGHT_CHASE_GRAY(-0.27),
    HEARTBEAT_RED(-0.25),
    HEARTBEAT_BLUE(-0.23),
    HEARTBEAT_WHITE(-0.21),
    HEARTBEAT_GRAY(-0.19),
    BREATH_RED(-0.17),
    BREATH_BLUE(-0.15),
    BREATH_GRAY(-0.13),
    STROBE_RED(-0.11),
    STROBE_BLUE(-0.09),
    STROBE_GOLD(-0.07),
    STROBE_WHITE(-0.05),
    /*
     * CP1: Color 1 Pattern
     */
    CP1_END_TO_END_BLEND_TO_BLACK(-0.03),
    CP1_LARSON_SCANNER(-0.01),
    CP1_LIGHT_CHASE(+0.01),
    CP1_HEARTBEAT_SLOW(+0.03),
    CP1_HEARTBEAT_MEDIUM(+0.05),
    CP1_HEARTBEAT_FAST(+0.07),
    CP1_BREATH_SLOW(+0.09),
    CP1_BREATH_FAST(+0.11),
    CP1_SHOT(+0.13),
    CP1_STROBE(+0.15),
    /*
     * CP2: Color 2 Pattern
     */
    CP2_END_TO_END_BLEND_TO_BLACK(+0.17),
    CP2_LARSON_SCANNER(+0.19),
    CP2_LIGHT_CHASE(+0.21),
    CP2_HEARTBEAT_SLOW(+0.23),
    CP2_HEARTBEAT_MEDIUM(+0.25),
    CP2_HEARTBEAT_FAST(+0.27),
    CP2_BREATH_SLOW(+0.29),
    CP2_BREATH_FAST(+0.31),
    CP2_SHOT(+0.33),
    CP2_STROBE(+0.35),
    /*
     * CP1_2: Color 1 and 2 Pattern
     */
    CP1_2_SPARKLE_1_ON_2(+0.37),
    CP1_2_SPARKLE_2_ON_1(+0.39),
    CP1_2_COLOR_GRADIENT(+0.41),
    CP1_2_BEATS_PER_MINUTE(+0.43),
    CP1_2_END_TO_END_BLEND_1_TO_2(+0.45),
    CP1_2_END_TO_END_BLEND(+0.47),
    CP1_2_NO_BLENDING(+0.49),
    CP1_2_TWINKLES(+0.51),
    CP1_2_COLOR_WAVES(+0.53),
    CP1_2_SINELON(+0.55),
    /*
     * Solid color
     */
    HOT_PINK(+0.57),
    DARK_RED(+0.59),
    RED(+0.61),
    RED_ORANGE(+0.63),
    ORANGE(+0.65),
    GOLD(+0.67),
    YELLOW(+0.69),
    LAWN_GREEN(+0.71),
    LIME(+0.73),
    DARK_GREEN(+0.75),
    GREEN(+0.77),
    BLUE_GREEN(+0.79),
    AQUA(+0.81),
    SKY_BLUE(+0.83),
    DARK_BLUE(+0.85),
    BLUE(+0.87),
    BLUE_VIOLET(+0.89),
    VIOLET(+0.91),
    WHITE(+0.93),
    GRAY(+0.95),
    DARK_GRAY(+0.97),
    BLACK(+0.99);

    public final double value;
    private BlinkinPattern(double value) {
      this.value = value;
    }
  };

  private static NemesisLED m_controller = null;
  private static Spark m_blinkin;
  private static BlinkinPattern m_currentPattern;
  private static HashMap<Alliance, BlinkinPattern[]> m_allianceColors = new HashMap<Alliance, BlinkinPattern[]>();
  private static final BlinkinPattern[] RED_ALLIANCE_PATTERNS = {
    BlinkinPattern.RED,
    BlinkinPattern.BREATH_RED,
    BlinkinPattern.LIGHT_CHASE_RED,
    BlinkinPattern.SHOT_RED,
    BlinkinPattern.STROBE_RED
  };
  private static final BlinkinPattern[] BLUE_ALLIANCE_PATTERNS = {
    BlinkinPattern.BLUE,
    BlinkinPattern.BREATH_BLUE,
    BlinkinPattern.LIGHT_CHASE_BLUE,
    BlinkinPattern.SHOT_BLUE,
    BlinkinPattern.STROBE_BLUE
  };

  private NemesisLED() {
    m_blinkin = new Spark(Constants.BLINKIN_LED_CONTROLLER_PORT);

    m_allianceColors.put(Alliance.Red, RED_ALLIANCE_PATTERNS);
    m_allianceColors.put(Alliance.Blue, BLUE_ALLIANCE_PATTERNS);
  }

  /**
   * Get instance of BlinkinLEDController
   * @return BlinkinLEDController object
   */
  public static NemesisLED getInstance() {
    if (m_controller == null) m_controller = new NemesisLED();
    return m_controller;
  }

  /**
   * Set LED pattern
   * @param pattern Desired LED light pattern
   */
  public void setPattern(BlinkinPattern pattern) {
    m_currentPattern = pattern;
    m_blinkin.set(m_currentPattern.value);
  }

  /**
   * Set LEDs alliance color solid pattern
   */
  public void setAllianceColorSolid() {
    setPattern(m_allianceColors.get(DriverStation.getAlliance())[0]);
  }

  /**
   * Set LEDs to alliance color breath pattern
   */
  public void setAllianceColorBreath() {
    setPattern(m_allianceColors.get(DriverStation.getAlliance())[1]);
  }

  /**
   * Set LEDs to alliance color chase pattern
   */
  public void setAllianceColorChase() {
    setPattern(m_allianceColors.get(DriverStation.getAlliance())[2]);
  }

  /**
   * Set LEDs to alliance color shot pattern
   */
  public void setAllianceColorShot() {
    setPattern(m_allianceColors.get(DriverStation.getAlliance())[3]);
  }

  /**
   * Set LEDs to alliance color strobe pattern
   */
  public void setAllianceColorStrobe() {
    setPattern(m_allianceColors.get(DriverStation.getAlliance())[4]);
  }

  /**
   * Set LEDs to team color
   */
  public void setTeamColor() {
    setPattern(BlinkinPattern.DARK_GREEN);
  }

  /**
   * Get current LED pattern
   * @return current LED pattern
   */
  public BlinkinPattern getCurrentPattern() {
    return m_currentPattern;
  }

  /**
   * Turn off LEDs
   */
  public void off() {
    setPattern(BlinkinPattern.BLACK);
  }
}


/**package frc.robot.subsystems.LEDS;

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
  private int length;
  //private int hue = 0;
  
    public NemesisLED(int port, int length) {
      // Initialize the LED hardware
      led = new AddressableLED(port);
      buffer = new AddressableLEDBuffer(length);
      led.setLength(length);
      led.start();
  
      basePattern = LEDPattern.kOff;
  
      setDefaultCommand(runOnce(() -> LEDPattern.kOff.applyTo(buffer)));
    }
  
    public void setRGB(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
        buffer.setRGB(i, g, r, b);
      }
      led.setData(buffer);
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

  public Command setWhite(Color color) {
      LEDPattern white = LEDPattern.solid(Color.kWhite);
      white.applyTo(buffer);
      led.setData(buffer);
      return Commands.none();

  }

  public Command setRed(Color color) {
      LEDPattern red = LEDPattern.solid(Color.kRed);
      red.applyTo(buffer);
      led.setData(buffer);
      return Commands.none();

  }

  public Command setOrange(Color color) {
      LEDPattern orange = LEDPattern.solid(Color.kOrange);
      orange.applyTo(buffer);
      led.setData(buffer);
      return Commands.none();
      
  }

  public Command setYellow(Color color) {
      LEDPattern yellow = LEDPattern.solid(Color.kYellow);
      yellow.applyTo(buffer);
      led.setData(buffer);
      return Commands.none();
      
  }

  public void setGreen(Color color) {
      LEDPattern green = LEDPattern.solid(Color.kGreen);
      green.applyTo(buffer);
      led.setData(buffer);
      //return Commands.none();
      
  }

  public Command setBlue(Color color) {
      LEDPattern blue = LEDPattern.solid(Color.kBlue);
      blue.applyTo(buffer);
      led.setData(buffer);
      return Commands.none();
      
  }

  public Command setPurple(Color color) {
      LEDPattern purple = LEDPattern.solid(Color.kPurple);
      purple.applyTo(buffer);
      led.setData(buffer);
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
 */
