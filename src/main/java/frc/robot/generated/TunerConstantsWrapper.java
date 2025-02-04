package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import lombok.Getter;

public class TunerConstantsWrapper {
  @Getter public CANBus kCANBus;
  @Getter public CANBus pigeonCanbusName;
  @Getter public LinearVelocity kSpeedAt12Volts;
  @Getter public SwerveDrivetrainConstants DrivetrainConstants;

  @Getter
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;

  @Getter
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;

  @Getter
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;

  @Getter
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  @Getter public double driveBaseRadius;
  @Getter public double odometryFrequency;

  public TunerConstantsWrapper() {
    switch (Constants.currentMode) {
      case KRONOS:
        kCANBus = TunerConstantsKronos.kCANBus;
        kSpeedAt12Volts = TunerConstantsKronos.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsKronos.DrivetrainConstants;
        FrontLeft = TunerConstantsKronos.FrontLeft;
        FrontRight = TunerConstantsKronos.FrontRight;
        BackLeft = TunerConstantsKronos.BackLeft;
        BackRight = TunerConstantsKronos.BackRight;
        driveBaseRadius =
            Math.max(
                Math.max(
                    Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
                    Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
                Math.max(
                    Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
                    Math.hypot(BackRight.LocationX, BackRight.LocationY)));
        odometryFrequency = new CANBus(kCANBus.getName()).isNetworkFD() ? 250.0 : 100.0;
        pigeonCanbusName = TunerConstantsKronos.kCANBus;
        break;
      case LARRY:
        kCANBus = TunerConstantsLarry.kCANBus;
        kSpeedAt12Volts = TunerConstantsLarry.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsLarry.DrivetrainConstants;
        FrontLeft = TunerConstantsLarry.FrontLeft;
        FrontRight = TunerConstantsLarry.FrontRight;
        BackLeft = TunerConstantsLarry.BackLeft;
        BackRight = TunerConstantsLarry.BackRight;
        driveBaseRadius =
            Math.max(
                Math.max(
                    Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
                    Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
                Math.max(
                    Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
                    Math.hypot(BackRight.LocationX, BackRight.LocationY)));
        odometryFrequency = new CANBus(kCANBus.getName()).isNetworkFD() ? 250.0 : 100.0;
        pigeonCanbusName = new CANBus("rio");
      case SIM:
        kCANBus = TunerConstantsLarry.kCANBus;
        kSpeedAt12Volts = TunerConstantsLarry.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsLarry.DrivetrainConstants;
        FrontLeft = TunerConstantsLarry.FrontLeft;
        FrontRight = TunerConstantsLarry.FrontRight;
        BackLeft = TunerConstantsLarry.BackLeft;
        BackRight = TunerConstantsLarry.BackRight;
        odometryFrequency = new CANBus(kCANBus.getName()).isNetworkFD() ? 250.0 : 100.0;
        driveBaseRadius =
            Math.max(
                Math.max(
                    Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
                    Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
                Math.max(
                    Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
                    Math.hypot(BackRight.LocationX, BackRight.LocationY)));
        break;
      default:
        break;
    }
  }
}
