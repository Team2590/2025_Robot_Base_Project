// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.generated.TunerConstantsLeonidas;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.FRCPolygon;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PolygonLocator;
// import frc.robot.util.PolygonLocator;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.Leonidas : simMode;
  public static final boolean tuningMode = true;
  public static final double loopPeriodSecs = 0.02;

  public static final double endEffectOffset = .2921; // Offset of the end effector to the

  public static class DriveToPoseConstraints {
    public static double maxVelocityMPS = 3;
    public static double maxAccelerationMPSSq = 3;
    public static double maxAngularVelocityRadPerSec = 6;
    public static double maxAngularAccelerationRadPerSecSq = 3;

    public static LoggedTunableNumber maxVelocityMPSScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxVelocityMPSScaler", 1);
    public static LoggedTunableNumber maxAccelerationMPSSqScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAccelerationMPSSqScaler", 1);
    public static LoggedTunableNumber maxAngularVelocityRadPerSecScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAngularVelocityRadPerSecScaler", 1);
    public static LoggedTunableNumber maxAngularAccelerationRadPerSecSqScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAngularAccelerationRadPerSecSqScaler", 1);

    public static PathConstraints fastpathConstraints =
        new PathConstraints(
            maxVelocityMPS,
            maxAccelerationMPSSq,
            maxAngularVelocityRadPerSec,
            maxAngularAccelerationRadPerSecSq);

    public static PathConstraints slowpathConstraints =
        new PathConstraints(
            maxVelocityMPS * maxVelocityMPSScaler.get(),
            maxAccelerationMPSSq * maxAccelerationMPSSqScaler.get(),
            maxAngularVelocityRadPerSec * maxAngularVelocityRadPerSecScaler.get(),
            maxAngularAccelerationRadPerSecSq * maxAngularAccelerationRadPerSecSqScaler.get());

    public static void updateTunableNumbers() {
      slowpathConstraints =
          new PathConstraints(
              maxVelocityMPS * maxVelocityMPSScaler.get(),
              maxAccelerationMPSSq * maxAccelerationMPSSqScaler.get(),
              maxAngularVelocityRadPerSec * maxAngularVelocityRadPerSecScaler.get(),
              maxAngularAccelerationRadPerSecSq * maxAngularAccelerationRadPerSecSqScaler.get());
    }
  }

  private static List<FRCPolygon> polygons = new ArrayList<>();
  private static Rectangle2D fieldBounds = new Rectangle2D.Double(0, 0, 15, 15);
  /*
   *
   * The ordering of points matters. I went mad trying to debug at one point,
   * turns out it was in a bowtie and not a square
   * Bowtie shape (incorrect rectangle):
   *
   * (0,2)---(2,2)
   * / \ / \
   * / \ / \
   * (0,0)---(2,0)
   *
   * Square shape (correct rectangle):
   *
   * (0,2)---(2,2)
   * | |
   * | |
   * (0,0)---(2,0)
   */
  public static final FRCPolygon playBox =
      new FRCPolygon(
          "playBox",
          new Translation2d(0, 0),
          new Translation2d(2, 0),
          new Translation2d(2, 2),
          new Translation2d(0, 2));

  public static final FRCPolygon reefBounds = new FRCPolygon("reef", "Reefbounds");
  public static final FRCPolygon BargeBoundsTop = new FRCPolygon("BargeTop", "BargeTopBound");
  public static final FRCPolygon BargeBoundsBot = new FRCPolygon("BargeBot", "BargeBotBound");
  public static final FRCPolygon PresetAlgae1 = new FRCPolygon("PresetAlgae1", "PresetAlgae1Bound");
  public static final FRCPolygon PresetAlgae2 = new FRCPolygon("PresetAlgae2", "PresetAlgae2Bound");
  public static final FRCPolygon PresetAlgae3 = new FRCPolygon("PresetAlgae3", "PresetAlgae3Bound");
  public static final FRCPolygon Processor = new FRCPolygon("Processor", "ProcessorBound");
  public static final FRCPolygon FeederStationTop =
      new FRCPolygon("FeederStationTop", "Station1Bound");
  public static final FRCPolygon FeederStationBot =
      new FRCPolygon("FeederStationBot", "Station2Bound");

  // Two ways to instantiate the polygons, this static initialization box is
  // necessary
  static {
    polygons.add(playBox);
    polygons.add(reefBounds);
    polygons.add(BargeBoundsTop);
    polygons.add(BargeBoundsBot);
    polygons.add(FeederStationTop);
    polygons.add(FeederStationBot);
    polygons.add(PresetAlgae1);
    polygons.add(PresetAlgae2);
    polygons.add(PresetAlgae3);
    polygons.add(Processor);
  }

  public static PolygonLocator locator = new PolygonLocator(polygons, fieldBounds);

  public static final String CANBUS = "Takeover";

  public final class ArmConstantsLarry {
    // Fill in
    public static final double HOME_SETPOINT = .155;
    public static final double CLIMB_SETPOINT = .198;
    public static final double GROUND_INTAKE_SETPOINT = 0;
    public static final double CORAL_STATION_INTAKE_SETPOINT = -0.1;
    public static final double REEF_1_SETPOINT = 0.01;
    public static final double REEF_2_3_SETPOINT = 0.06;
    public static final double REEF_4_SETPOINT = 0.15;
    public static final double BARGE = -0.15;
    public static final int ARM_CAN_ID = 45;
    public static final int ARM_CANCODER_ID = 44;
    public static final double ARM_GEAR_RATIO = 266.67;
    public static final double MAG_OFFSET = 0;
    public static final double ARM_MAX = -0.35; // -.3
    public static final String CANBUS = "Takeover";
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERT = true;
    public static final boolean BRAKE = true;
    public static final double REDUCTION = 1;
    public static final double SENSOR_REDUCTION = 1;
  }

  public final class ArmConstantsKronos {
    // Fill in, these are the same as Larry. Once update, please delete this comment
    public static final double HOME_SETPOINT = .155;
    public static final double CLIMB_SETPOINT = .198;
    public static final double GROUND_INTAKE_SETPOINT = 0;
    public static final double CORAL_STATION_INTAKE_SETPOINT = -0.1;
    public static final double REEF_1_SETPOINT = 0.01;
    public static final double REEF_2_3_SETPOINT = 0.06;
    public static final double REEF_4_SETPOINT = 0.15;
    public static final double BARGE = -0.15;
    public static final int ARM_CAN_ID = 45;
    public static final int ARM_CANCODER_ID = 44;
    public static final double ARM_GEAR_RATIO = 266.67;
    public static final double MAG_OFFSET = -.156;
    public static final double ARM_MAX = -0.35; // -.3
    public static final String CANBUS = "Takeover";
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERT = true;
    public static final boolean BRAKE = true;
    public static final double REDUCTION = 1;
    public static final double SENSOR_REDUCTION = 1;
  }

  public static class ElevatorConstantsLarry {
    static int canID = 1;
    static String canBus = "Takeover";
    static int currentLimitAmps = 40;
    static boolean invert = true;
    static boolean brake = true;
    static double reduction = 1;
  }

  public static class CoralPose {
    public final double x;
    public final double y;
    public final double elevatorPosition;
    public final double rotationTarget;
    public static final Transform2d tagToLeftReef =
        new Pose2d(3.18, 4.06, new Rotation2d(Math.toRadians(-2.03)))
            .minus(VisionConstants.aprilTagLayout.getTagPose(18).get().toPose2d());
    public static final Transform2d tagToRightReef =
        new Pose2d(3.18, 3.78, new Rotation2d(Math.toRadians(-0.53)))
            .minus(VisionConstants.aprilTagLayout.getTagPose(18).get().toPose2d());

    public CoralPose(double x, double y, double elevatorPosition, double rotationTarget) {
      this.x = x;
      this.y = y;
      this.elevatorPosition = elevatorPosition;
      this.rotationTarget = rotationTarget;
    }

    public CoralPose(double x, double y) {
      this.x = x;
      this.y = y;
      this.elevatorPosition = 0;
      this.rotationTarget = 0;
    }
  }

  public class DriveToPoseConstants {
    public static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstantsLeonidas.kSpeedAt12Volts;
    public static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.25);
    public static LinearVelocity MAX_DRIVE_TO_POSE_TRANSLATION_VELOCITY =
        MAX_TELEOP_VELOCITY.div(2.0);
    public static final LinearAcceleration MAX_DRIVE_TO_POSE_TRANSLATION_ACCELERATION =
        MetersPerSecondPerSecond.of(2.0);
    public static final AngularVelocity MAX_DRIVE_TO_POSE_ANGULAR_VELOCITY =
        MAX_TELEOP_ANGULAR_VELOCITY.times(0.75);
    public static final AngularAcceleration MAX_DRIVE_TO_POSE_ANGULAR_ACCELERATION =
        RadiansPerSecondPerSecond.of(6.0 * Math.PI);
    public static final double THETA_kD = 0;
    public static final double THETA_kI = 0;
    public static final double THETA_kP = 3.0;
    public static final double X_kD = 0;
    public static final double X_kI = 0;
    public static final double X_kP = 5;
    public static final double Y_kD = 0;
    public static final double Y_kI = 0;
    public static final double Y_kP = 5;
  }

  public static class ArmConstantsLeonidas {
    public static final double ARM_FACTORY_SAFETY_MIN = -.1;
    public static final double ARM_FACTORY_SAFETY_MAX = .8;
    public static final int canID = 1;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 60;
    public static final boolean invert = true;
    public static final boolean brake = true;
    public static final double reduction = 1; // was a 94.18
    public static final int cancoderID = 5;
    public static final double ARM_SET_STOW = .25; // .33; // TODO
    public static final double ARM_VERTICAL_POS = .25;
    // public static final double magOffset = -.596436; // -.398
    // WHEN RESETTING ARM, RESET TO 6 DEGREES FROM HORIZONTAL (use measur;e app on iPhone :) )
    public static final double magOffset = 0.385; // 0; // .454; // 0; // -.473;
    // .146; // .142; // -0.15; // -0.379; // -.3347; // -.635; // TODO
    public static final double sensorReduction = 58.8; // TODO
    public static final double BACK_HORIZONTAL = 0.5;
    public static final double ARM_OPERATIONAL_MIN_POS = -.5;
    public static final double ARM_OPERATIONAL_MAX_POS = 2;
    public static final double ARM_RELEASE_DIFF = 0.078;
    public static final double ARM_SCORING_CORAL_POS_L4 = ARM_SET_STOW; // .TODO
    public static final double ARM_SCORING_CORAL_POSE_L4_POST = 0;
    public static final double ARM_INTAKE_SOURCE_POSITION = .18; // TODO
    public static final double ARM_DEALGAE_POSITION = 0.012;
    public static final double ARM_DEALGAE_POST = 0;
    public static final double ARM_DEALGAE_PRE = 0;
    public static final double ARM_INTAKE_ALGAE_POS = .53;
    public static final double ARM_BARGE_FRONT_FRONT_POS = .175; // TODO: find real value
    public static final double ARM_BARGE_FRONT_BACK_POS = .325; // TODO: find real value
    public static final double ARM_BARGE_BACK_FRONT_POS = 1.175; // TODO: find real value
    public static final double ARM_BARGE_BACK_BACK_POS = 1.325; // TODO: find real valu
    // public static final double ARM_SCORING_POS = .7; // TODO: find real value
    public static final double ARM_WRAP_POS = 0.794;
    public static final double ARM_HANDOFF_POS = 0.75;
    public static final double ARM_FRONT_THRESHOLD_POS = .63;
    public static final double ARM_BACK_THRESHOLD_POS = .87;
    public static final double ARM_STOW_FRONT = 0.25;
    public static final double ARM_SCORE_FRONT_FRONT_PRE = 0.15;
    public static final double ARM_SCORE_FRONT_FRONT_POST = 0.015;
    public static final double ARM_DEALGAE_FRONT_FRONT = 0;
    public static final double ARM_SCORE_FRONT_BACK_PRE = 0.35;
    public static final double ARM_SCORE_FRONT_BACK_POST = 0.485;
    public static final double ARM_DEALGAE_FRONT_BACK = 0.5;
    public static final double ARM_STOW_BACK = 1.25;
    public static final double ARM_SCORE_BACK_FRONT_PRE = 1.15;
    public static final double ARM_SCORE_BACK_FRONT_POST = 1.015;
    public static final double ARM_DEALGAE_BACK_FRONT = 1;
    public static final double ARM_SCORE_BACK_BACK_PRE = 1.35;
    public static final double ARM_SCORE_BACK_BACK_POST = 1.485;
    public static final double ARM_DEALGAE_BACK_BACK = 1.5;
    public static final double ARM_PICKUP_FRONT_BACK_POS = 0.53;
    public static final double ARM_PICKUP_BACK_BACK_POS = 1.53;
  }

  public static class ElevatorConstantsLeonidas {
    public static final double OFFSET = 0;
    public static final double ELEVATOR_OPERATIONAL_MIN_POS = -0.1;
    public static final double ELEVATOR_OPERATIONAL_MAX_POS = 27.5;
    public static final int canID = 25;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120;
    public static final boolean invert = true;
    public static final boolean brake = true;
    public static final double reduction = 7;
    public static final int followerCanID = 26;
    public static final String followerCanBus = "Takeover";
    public static final boolean followerOpposeLeader = false;
    public static final double kS = 0.59545; // TODO
    public static final double kV = 0.1609; // TODO
    public static final double ELEVATOR_HANDOFF_POS = 16.75;
    public static final double ELEVATOR_HANDOFF_PRE_POS = 14.2;
    public static final double ELEVATOR_PROCESSOR_POS = 1 + 1.5 + 1.5;
    public static final double ELEVATOR_STOW_POS = 1;
    public static final double ELEVATOR_L2_POS = 3; // TODO
    public static final double ELEVATOR_L3_POS = 11.35; // TODO
    public static final double ELEVATOR_L4_POS = 25.0; // TODO
    public static final double ELEVATOR_DEALGAE_L2 = 12; // TODO
    public static final double ELEVATOR_DEALGAE_L3 = 19.76; // TODO
    public static final double ELEVATOR_SOURCE_POS = 5.15; // TODO
    public static final double ELEVATOR_MANUAL_VOLTAGE = 1; // TODO
    public static final double ELEVATOR_BARGE_POS = 27.5; // TODO: find real value
    public static final double ELEVATOR_INTAKE_ALGAE_POS = 0; // TODO find real value
    public static final double ELEVATOR_HANDOFF_TRANSITION_POS = ELEVATOR_HANDOFF_POS + 1.5;
    public static final double MIN_ELEVATOR_HEIGHT_ARM_PARALLEL = 0;
  }

  /*   public final class ElevatorConstantsLeonidas {
    public static double ELEVATOR_FACTORY_MIN_POS = 5; // TODO: change to actual value
    public static double ELEVATOR_FACTORY_MAX_POS = 89.5; // TODO: change to actual value
    public static final int canID = 25;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 40; // TODO
    public static final boolean invert = false;
    public static final boolean brake = true;
    public static final double reduction = 7;
    public static final double kS = 0.22720;
    public static final double kV = 0.14051;
    public static final double l2 = 27.210938;
    public static final double l3 = 51.997559;
    public static final double l4 = 84.572266;
  }
  */

  public final class EndEffectorConstantsLeonidas {
    public static final int canID = 10;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120; // TODO
    public static final boolean invert = false; // TODO
    public static final boolean brake = false; // TODO
    public static final double reduction = 1;
    public static final double INTAKE_VOLTAGE = 12;
    public static final double OUTTAKE_VOLTAGE = -12;
    public static final double INTAKE_ALGAE_VOLTAGE = -12;
    public static final double HOLD_ALGAE_VOLTAGE = -0.75;
    public static final double HOLD_CORAL_VOLTAGE = -.2;
    public static final int PROX_CHANNEL = 0;
  }

  public final class IntakeConstantsLeonidas {
    public static final int canID = 50;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 40;
    public static final boolean invert = true;
    public static final boolean brake = true;
    public static final double reduction = 1;
    public static final int followerCanID = 51;
    public static final String followerCanBus = "Takeover";
    public static final boolean followerOpposeLeader = false;
    public static final int PROX_ONE_CHANNEL = 1; // TODO: real value
    public static final int PROX_TWO_CHANNEL = 2; // TODO: real value
    public static final double INTAKE_CORAL_INTAKE_SPEED = -12; // TODO
    public static final double INTAKE_CORAL_OUTTAKE_SPEED = 11; // TODO
    public static final double HAS_ALGAE_THRESHOLD_CURRENT = 10; // TODO
  }

  public final class IntakeArmConstantsLeonidas {
    public static final double INTAKE_GROUND_CORAL_POS = .41;
    public static final double INTAKE_HANDOFF_POS = .018;
    public static final double INTAKE_HOME_POS = .15;
    public static final double L1_POS = .15;
    public static final int canID = 52;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 40;
    public static final boolean invert = true;
    public static final boolean brake = true;
    public static final double reduction = 19.16;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double INTAKE_OPERATIONAL_MIN_POS = -.1;
    public static final double INTAKE_OPERATIONAL_MAX_POS = .5;
  }

  public final class ClimbConstantsLeonidas {
    public static final int canID = 14;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120; // TODO
    public static final boolean invert = false;
    public static final boolean brake = false;
    public static final double reduction = 1;
    public static final double CLIMB_MECHANISM_POSITION = 47.2; // 13
    public static final double CLIMB_MAX_POSITION = 220;
    public static final double CLIMB_VOLTAGE = 11.0; // 2.0 tested
  }

  public final class LEDConstantsLeonidas {
    public static final int port = 0;
    public static final int length = 56;
    public static final int halfWay = 29;
    public static final Color startColor = Color.kRed;
    public static final Color endColor = Color.kWhite;
    public static final double blinkTime = 0.2;
  }

  public static class IntakeConstantsLarry {
    static int canID = 60;
    static int currentLimitAmps = 40;
    static String canBus = "Takeover";
    static boolean invert = false;
    static double reduction = 1;
    static boolean brake = true;
  }

  public static class DriveToPoseStraight {

    public static double maxVelocityMPS = 3;
    public static double maxAngularVelocityRadPerSec = 6;

    public static class XController {
      public static double kP = 2.25;
      public static double kD = 0;
      public static double tolerance = 0.01; // ~.4 inches
    }

    public static class YController {
      public static double kP = 2.25;
      public static double kD = 0;
      public static double tolerance = 0.01;
    }

    public static class ThetaController {
      public static double kP = 3.5;
      public static double kD = 0;
      public static double tolerance = Units.degreesToRadians(1.0);
    }
  }

  public static final int BLINKIN_LED_CONTROLLER_PORT = 0; // TODO

  public enum LEDPattern {
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

    private LEDPattern(double value) {
      this.value = value;
    }
  };

  public static enum Mode {
    /** Running on a real robot. */
    COMP,

    KRONOS,

    LARRY,

    Leonidas,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
