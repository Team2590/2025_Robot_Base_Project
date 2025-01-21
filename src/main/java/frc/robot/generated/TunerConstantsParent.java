package frc.robot.generated;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

public class TunerConstantsParent {
  /**
   * Canbus (drivetrainconstants) pigeon2ID (drivetrainconstants) frontleft frontright backleft
   * backright kspeedvoltsat12
   */
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;

  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;
  public LinearVelocity kSpeedAt12Volts;
  public SwerveDrivetrainConstants DrivetrainConstants;

  public TunerConstantsParent(Constants.Mode mode) {
    try {
      switch (mode) {
        case KRONOS:
          FrontLeft =
              (SwerveModuleConstants<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                  TunerConstantsKronos.class.getField("FrontLeft").get(null);
          FrontRight =
              (SwerveModuleConstants<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                  TunerConstantsKronos.class.getField("FrontRight").get(null);
          BackLeft =
              (SwerveModuleConstants<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                  TunerConstantsKronos.class.getField("BackLeft").get(null);
          BackRight =
              (SwerveModuleConstants<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                  TunerConstantsKronos.class.getField("BackRight").get(null);
          kSpeedAt12Volts =
              (LinearVelocity)
                  TunerConstantsKronos.class
                      .getField("kSpeedAt12Volts")
                      .get(null); // .get(LinearVelocity);
          DrivetrainConstants =
              (SwerveDrivetrainConstants)
                  TunerConstantsKronos.class.getField("DrivetrainConstants").get(null);
          break;

        case LEO:
          break;

        case COMP:
          break;

        default:
          break;
      }

    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("womp womp");
    }
  }

  public SwerveModuleConstants[] getSwerveModuleConstants() {
    SwerveModuleConstants[] modules = {FrontLeft, FrontRight, BackLeft, BackRight};
    return modules;
  }

  public LinearVelocity getLinearVelocity() {
    return kSpeedAt12Volts;
  }

  public SwerveDrivetrainConstants getDrivetrainConstants() {
    return DrivetrainConstants;
  }
}
