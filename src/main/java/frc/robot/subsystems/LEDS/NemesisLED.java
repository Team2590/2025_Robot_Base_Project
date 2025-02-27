package frc.robot.subsystems.LEDS;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.LEDPattern;


/** Control REV Robotics Blinkin LED controller */
public class NemesisLED extends SubsystemBase{
  private static Spark m_blinkin;

  public NemesisLED(int id) {
    m_blinkin = new Spark(id);
  }

  /**
   * Set LED pattern
   * @param pattern Desired LED light pattern
   */
  public Command setPattern(LEDPattern pattern) {
    return runOnce(() -> m_blinkin.set(pattern.value));
  }

  /**
   * Turn off LEDs
   */
  public Command off() {
    return runOnce(() -> setPattern(LEDPattern.BLACK));
  }
}