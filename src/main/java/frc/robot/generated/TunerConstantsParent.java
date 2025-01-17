package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public class TunerConstantsParent {
  private static Slot0Configs steerGains;
  private static Slot0Configs driveGains;
  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the drive motor
  private static SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
  private static SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static Current kSlipCurrent = Amps.of(120.0);

  // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  private static CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static Pigeon2Configuration pigeonConfigs = null;
  public static CANBus kCANBus;
  public static LinearVelocity kSpeedAt12Volts;
  private static double kCoupleRatio;
  private static double kDriveGearRatio;
  private static double kSteerGearRatio;
  private static Distance kWheelRadius;
  private static boolean kInvertLeftSide;
  private static boolean kInvertRightSide;
  private static int kPigeonId;
  // These are only used for simulation
  private static MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  // Simulated voltage necessary to overcome friction
  private static Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static Voltage kDriveFrictionVoltage = Volts.of(0.2);
  public static SwerveDrivetrainConstants DrivetrainConstants;
  private static SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator;
  // Front Left
  private static int kFrontLeftDriveMotorId;
  private static int kFrontLeftSteerMotorId;
  private static int kFrontLeftEncoderId;
  private static Angle kFrontLeftEncoderOffset;
  private static boolean kFrontLeftSteerMotorInverted;
  private static boolean kFrontLeftEncoderInverted;

  private static Distance kFrontLeftXPos;
  private static Distance kFrontLeftYPos;

  // Front Right
  private static int kFrontRightDriveMotorId;
  private static int kFrontRightSteerMotorId;
  private static int kFrontRightEncoderId;
  private static Angle kFrontRightEncoderOffset;
  private static boolean kFrontRightSteerMotorInverted;
  private static boolean kFrontRightEncoderInverted;

  private static Distance kFrontRightXPos;
  private static Distance kFrontRightYPos;

  // Back Left
  private static int kBackLeftDriveMotorId;
  private static int kBackLeftSteerMotorId;
  private static int kBackLeftEncoderId;
  private static Angle kBackLeftEncoderOffset;
  private static boolean kBackLeftSteerMotorInverted;
  private static boolean kBackLeftEncoderInverted;

  private static Distance kBackLeftXPos;
  private static Distance kBackLeftYPos;

  // Back Right
  private static int kBackRightDriveMotorId;
  private static int kBackRightSteerMotorId;
  private static int kBackRightEncoderId;
  private static Angle kBackRightEncoderOffset;
  private static boolean kBackRightSteerMotorInverted;
  private static boolean kBackRightEncoderInverted;

  private static Distance kBackRightXPos;
  private static Distance kBackRightYPos;

  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  // public static CommandSwerveDrivetrain createDrivetrain() {
  //     return new CommandSwerveDrivetrain(
  //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
  //     );
  // }

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
