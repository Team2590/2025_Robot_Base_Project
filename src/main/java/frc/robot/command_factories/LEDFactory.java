package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDFactory {
  public static void init() {
    RobotContainer.getLed()
        .setColorVoid(
            Constants.LEDConstantsLeonidas.startColor, Constants.LEDConstantsLeonidas.endColor);
  }

  public static Command solid() {
    return RobotContainer.getLed()
        .setColor(
            Constants.LEDConstantsLeonidas.startColor, Constants.LEDConstantsLeonidas.endColor);
  }

  public static Command blink() {
    return RobotContainer.getLed()
        .setBlink(
            Constants.LEDConstantsLeonidas.startColor,
            Constants.LEDConstantsLeonidas.endColor,
            Constants.LEDConstantsLeonidas.blinkTime);
  }

  public static Command auraRizz() {
    return RobotContainer.getLed()
        .setAuraRizz(
            Constants.LEDConstantsLeonidas.startColor, Constants.LEDConstantsLeonidas.endColor);
  }

  public static Command nemesisFlow() {
    return RobotContainer.getLed()
        .setNemesisFlow(
            Constants.LEDConstantsLeonidas.startColor, Constants.LEDConstantsLeonidas.endColor);
  }

  public static Command progressMask() {
    return RobotContainer.getLed()
        .setProgressMask(
            Constants.LEDConstantsLeonidas.startColor, Constants.LEDConstantsLeonidas.endColor);
  }
}
