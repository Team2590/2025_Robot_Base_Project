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

  public static class ElevatorConstantsLeonidas {
    public static double ELEVATOR_OPERATIONAL_MIN_POS = 5; // TODO: change to actual value
    public static double ELEVATOR_OPERATIONAL_MAX_POS = 10; // TODO: change to actual value
    public static double ELEVATOR_SAFETY_POS = 7.5; // TODO: change to actual value
    public static double ELEVATOR_L2_POS = 6; // TODO: change to actual value
    public static double ELEVATOR_L3_POS = 7; // TODO: change to actual value
    public static double ELEVATOR_L4_POS = 8; // TODO: change to actual value
  }

  public static class ArmConstantsLeonidas {
    public static double ARM_OPERATIONAL_MIN_POS = 5; // TODO: change to actual value
    public static double ARM_OPERATIONAL_MAX_POS = 10; // TODO: change to actual value
    public static double ARM_SAFETY_MIN_POS = 5; // TODO: change to actual value
    public static double ARM_SAFETY_MAX_POS = 10; // TODO: change to actual value
    public static double ARM_L2_POS = 6; // TODO: change to actual value
    public static double ARM_L3_POS = 7; // TODO: change to actual value
    public static double ARM_L4_POS = 8; // TODO: change to actual value
  }

  public static class EndEffectorConstantsLeonidas {
    static int canID = 2; // TODO
    static String canBus = "Takeover";
    static int currentLimitAmps = 120; // TODO
    static boolean invert = true; // TODO
    static boolean brake = true; // TODO
    static double reduction = 1; // TODO
  }

  public static class IntakeConstantsLeonidas {
    public static double INTAKE_FACTORY_CORAL_POSITION = 10; // TODO: Find Actual Value
    public static double INTAKE_FACTORY_ALGAE_POSITION = 4; // TODO: Find Actual Value
    public static double INTAKE_FACTORY_HOME_POSITION = 0; // TODO: Find Actual Value
    public static double INTAKE_FACTORY_HOLDING_ALGAE_POSITION = 2; // TODO: Find Actual Value
    public static double INTAKE_CORAL_OUTTAKE_SPEED = -2; // TODO: Find Actual Value
    public static double INTAKE_ALGAE_OUTTAKE_SPEED = -1; // TODO: Find Actual Value
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
