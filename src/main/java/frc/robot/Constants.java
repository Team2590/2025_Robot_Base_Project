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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.FRCPolygon;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PolygonLocator;
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
    public static double maxVelocityMPS = 0;
    public static double maxAccelerationMPSSq = 0;
    public static double maxAngularVelocityRadPerSec = 0;
    public static double maxAngularAccelerationRadPerSecSq = 0;

    public static PathConstraints pathConstraints =
        new PathConstraints(
            maxVelocityMPS,
            maxAccelerationMPSSq,
            maxAngularVelocityRadPerSec,
            maxAngularAccelerationRadPerSecSq);
  }

  private static List<FRCPolygon> polygons = new ArrayList<>();
  private static Rectangle2D fieldBounds = new Rectangle2D.Double(0, 0, 15, 15);
  /*
   *
   * The ordering of points matters. I went mad trying to debug at one point, turns out it was in a bowtie and not a square
   * Bowtie shape (incorrect rectangle):
   *
   *     (0,2)---(2,2)
   *      / \   / \
   *     /   \ /   \
   *    (0,0)---(2,0)
   *
   * Square shape (correct rectangle):
   *
   *    (0,2)---(2,2)
   *     |       |
   *     |       |
   *    (0,0)---(2,0)
   */
  public static final FRCPolygon playBox =
      new FRCPolygon(
          "playBox",
          new Translation2d(0, 0),
          new Translation2d(2, 0),
          new Translation2d(2, 2),
          new Translation2d(0, 2));

  public static final boolean flipside =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

  public static final FRCPolygon reefBounds = new FRCPolygon("reef", "Reefbounds");
  public static final FRCPolygon BargeBoundsTop = new FRCPolygon("BargeTop", "BargeTop");
  public static final FRCPolygon BargeBoundsBot = new FRCPolygon("BargeBot", "BargeBot");
  // public static final FRCPolygon PresetAlgae = new FRCPolygon("PresetAlgae", "FloatingAlgae");
  public static final FRCPolygon Processor = new FRCPolygon("Processor", "Processor");
  public static final FRCPolygon FeederStationTop = new FRCPolygon("FeederStationTop", "Station1");
  public static final FRCPolygon FeederStationBot = new FRCPolygon("FeederStationBot", "Station2");

  // Two ways to instantiate the polygons, this static initialization box is necessary
  static {
    polygons.add(playBox);
    polygons.add(reefBounds);
    polygons.add(BargeBoundsTop);
    polygons.add(BargeBoundsBot);
    // polygons.add(PresetAlgae);
    polygons.add(Processor);
  }

  public static PolygonLocator locator = new PolygonLocator(polygons, fieldBounds);

  public static LoggedTunableNumber homeSetpoint =
      new LoggedTunableNumber("Arm/IntakeSetpoint", .155);
  public static final String CANBUS = "Takeover";

  public final class ArmConstantsLarry {
    // Fill in
    public static final double HOME_SETPOINT = homeSetpoint.get();
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

  public final class ArmConstantsKronos {
    // Fill in, these are the same as Larry. Once update, please delete this comment
    public static final double HOME_SETPOINT = homeSetpoint.get();
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

  public static class ArmConstantsLeonidas {
    public static double ARM_FACTORY_SAFETY_MIN = -0.148;
    public static double ARM_FACTORY_SAFETY_MAX = .36;
    public static final int canID = 1;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 40;
    public static final boolean invert = false;
    public static final boolean brake = false;
    public static final double reduction = 1; // was a 94.18
    public static final int cancoderID = 5;
    public static final double magOffset = -.596436; // -.398
    public static final double sensorReduction = 58.8;
    public static double ARM_OPERATIONAL_MIN_POS = -.34;
    public static double ARM_OPERATIONAL_MAX_POS = .55;
    public static double ARM_DANGER_MIN_POS = ARM_OPERATIONAL_MIN_POS;
    public static double ARM_DANGER_MAX_POS = -0.14;
    public static double ARM_SCORING_CORAL_POS = 0.5; // TODO: change to actual value
    public static double ARM_INTAKE_SOURCE_POSITION = -0.34;
  }

  public static class ElevatorConstantsLeonidas {
    public static double ELEVATOR_OPERATIONAL_MIN_POS = 5;
    public static double ELEVATOR_OPERATIONAL_MAX_POS = 89.5;
    public static final int canID = 25;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120;
    public static final boolean invert = false;
    public static final boolean brake = true;
    public static final double reduction = 7;
    public static final double kS = 0.22720;
    public static final double kV = 0.14051;
    public static double ELEVATOR_DANGER_MIN_POS = ELEVATOR_OPERATIONAL_MIN_POS;
    public static double ELEVATOR_DANGER_MAX_POS = 27.28;
    public static double ELEVATOR_L2_POS = 27.210938;
    public static double ELEVATOR_L3_POS = 51.997559;
    public static double ELEVATOR_L4_POS = 84.572266;
    public static double ELEVATOR_SOURCE_POS = 42;
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
    public static final boolean brake = true; // TODO
    public static final double reduction = 1; // TODO
    public static final double INTAKE_VOLTAGE = 6;
    public static final double EJECT_VOLTAGE = -INTAKE_VOLTAGE;
  }

  public final class IntakeConstantsLeonidas {
    public static final int canID = 24;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120;
    public static final boolean invert = true;
    public static final boolean brake = true;
    public static final double reduction = 1;
    public static final double INTAKE_FACTORY_CORAL_POSITION = 0;
    public static final double INTAKE_FACTORY_ALGAE_POSITION = 5.4;
    public static final double INTAKE_FACTORY_HOME_POSITION = 0;
    public static final double INTAKE_FACTORY_HOLDING_ALGAE_POSITION = 0;
    public static final double INTAKE_CORAL_INTAKE_SPEED = 6;
    public static final double INTAKE_CORAL_OUTTAKE_SPEED = -8; // TODO
    public static final double INTAKE_ALGAE_INTAKE_SPEED = 6;
    public static final double INTAKE_ALGAE_OUTTAKE_SPEED = -8;
  }

  public final class IntakeArmConstantsLeonidas {
    public static final double INTAKE_CORAL_POS = 11.4;
    public static final double INTAKE_ALGAE_POS = 6.5;
    public static final int canID = 15;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120;
    public static final boolean invert = true;
    public static final boolean brake = true;
    public static final double reduction = 38.33;
    public static final double kS = 0.25242;
    public static final double kV = 0.34993;
  }

  public final class ClimbConstantsLeonidas {
    public static final int canID = 14;
    public static final String canBus = "Takeover";
    public static final int currentLimitAmps = 120; // TODO
    public static final boolean invert = false;
    public static final boolean brake = false;
    public static final double reduction = 1; // TODO
    public static final double CLIMB_MECHANISM_POSITION = 55.2;
    public static final double CLIMB_MAX_POSITION = 178; // 213.25 (actual)
    public static final double CLIMB_VOLTAGE = .1;
  }

  public static class IntakeConstantsLarry {
    static int canID = 60;
    static int currentLimitAmps = 40;
    static String canBus = "Takeover";
    static boolean invert = false;
    static double reduction = 1;
    static boolean brake = true;
  }

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
