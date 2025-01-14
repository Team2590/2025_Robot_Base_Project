// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  static {
    initializeConstants();
  }

  public static String canBusName;
  static final double ODOMETRY_FREQUENCY = new CANBus(canBusName).isNetworkFD() ? 250.0 : 100.0;
  public static double trackWidthX; // meters
  public static double trackWidthY; // meters
  public static double maxLinearSpeed; // meters
  public static double driveMotorGearReduction;
  public static double turnMotorGearReduction;
  public static double driveStatorCurrentLimit;
  public static double maxAngularSpeed;
  public static double driveBaseRadius;
  public static int pigeonId;

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(2.000);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxAngularSpeed, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsKronos = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(13)
        .turnMotorId(11)
        .encoderId(12)
        .encoderOffset(Rotation2d.fromRotations(0.069))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(23)
        .turnMotorId(21)
        .encoderId(22)
        .encoderOffset(Rotation2d.fromRotations(-1.805))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(33)
        .turnMotorId(31)
        .encoderId(32)
        .encoderOffset(Rotation2d.fromRotations(0.604))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(43)
        .turnMotorId(41)
        .encoderId(42)
        .encoderOffset(Rotation2d.fromRotations(1.977))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsLeo = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(2)
        .encoderId(1)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(0)
        .encoderId(0)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(6)
        .encoderId(3)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(4)
        .encoderId(2)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static void initializeConstants() {
    switch (Constants.getMode()) {
      case REAL:
        canBusName = "";
        trackWidthX = Units.inchesToMeters(18.75);
        trackWidthY = Units.inchesToMeters(18.75);
        maxLinearSpeed = Units.feetToMeters(15.5);
        driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
        maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
        driveMotorGearReduction = (50 / 14) * (17 / 27) * (45 / 15);
        turnMotorGearReduction = (150.0 / 7.0);
        driveStatorCurrentLimit = 80;
        break;
      case KRONOS:
        canBusName = "Takeover";
        trackWidthX = Units.inchesToMeters(18.75);
        trackWidthY = Units.inchesToMeters(18.75);
        maxLinearSpeed = Units.feetToMeters(15.5);
        driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
        maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
        driveMotorGearReduction = (50 / 14) * (17 / 27) * (45 / 15);
        turnMotorGearReduction = (150.0 / 7.0);
        driveStatorCurrentLimit = 80;
        break;
      case SIM:
        canBusName = "";
        trackWidthX = Units.inchesToMeters(18.75);
        trackWidthY = Units.inchesToMeters(18.75);
        maxLinearSpeed = Units.feetToMeters(15.5);
        driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
        maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
        driveMotorGearReduction = (50 / 14) * (17 / 27) * (45 / 15);
        turnMotorGearReduction = (150.0 / 7.0);
        driveStatorCurrentLimit = 30;
        break;
      case REPLAY:
        // ...
        break;
    }
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderId,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
