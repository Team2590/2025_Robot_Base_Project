// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  public static final double odometryFrequency = 250;
  public static final double trackWidthX = Units.inchesToMeters(20.75);
  public static final double trackWidthY = Units.inchesToMeters(20.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = 4.69 / driveBaseRadius;

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(2.000);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxAngularSpeed, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(2)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(0)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(6)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(4)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(2)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(0)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(6)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(4)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRotations(0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = 0;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
