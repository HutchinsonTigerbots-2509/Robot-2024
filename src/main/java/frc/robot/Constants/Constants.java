package frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import frc.robot.Commands.Drivetrain.CommandSwerveDrivetrain;

public class Constants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(50).withKI(0.5).withKD(1.5).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 300.0;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final double kSpeedAt12VoltsMps = 4.73;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 2.8333333333333335;

  private static final double kDriveGearRatio = 6.538461538461539;
  private static final double kSteerGearRatio = 11.314285714285715;
  private static final double kWheelRadiusInches = 1.937;

  public static final double kwheelCircumference = 12.4092909817;

  public static final boolean kSteerMotorReversed = true;
  public static final boolean kInvertLeftSide = true;
  public static final boolean kInvertRightSide = false;

  public static final boolean kInvertEncoder = false;

  private static final String kCANbusName = "";

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  // Simulated voltage necessary to overcome friction
  private static final double kSteerFrictionVoltage = 0.25;
  private static final double kDriveFrictionVoltage = 0.25;

  private static AHRS navx = new AHRS();

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withCANbusName(kCANbusName);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withSteerMotorInverted(kSteerMotorReversed);

  // Front Left
  public static final int kFrontLeftDriveMotorId = 4;
  public static final int kFrontLeftSteerMotorId = 5;
  public static final int kFrontLeftEncoderId = 24;
  public static final double kFrontLeftEncoderOffset = 0.418701171875;

  private static final double kFrontLeftXPosInches = 8.625;
  private static final double kFrontLeftYPosInches = 8.625;

  // Front Right
  public static final int kFrontRightDriveMotorId = 6;
  public static final int kFrontRightSteerMotorId = 7;
  public static final int kFrontRightEncoderId = 21;
  public static final double kFrontRightEncoderOffset = -0.015625;

  private static final double kFrontRightXPosInches = 8.625;
  private static final double kFrontRightYPosInches = -8.625;

  // Back Left
  public static final int kBackLeftDriveMotorId = 2;
  public static final int kBackLeftSteerMotorId = 3;
  public static final int kBackLeftEncoderId = 23;
  public static final double kBackLeftEncoderOffset = 0.04736328125;

  private static final double kBackLeftXPosInches = -8.625;
  private static final double kBackLeftYPosInches = 8.625;

  // Back Right
  public static final int kBackRightDriveMotorId = 0;
  public static final int kBackRightSteerMotorId = 1;
  public static final int kBackRightEncoderId = 22;
  public static final double kBackRightEncoderOffset = -0.349609375;

  private static final double kBackRightXPosInches = -8.625;
  private static final double kBackRightYPosInches = -8.625;

  // Arm
  public static final int kArmId = 16;
  public static final int kTopLimitSwitchID = 2;
  public static final int kBottomLimitSwitchID = 3;

  // Intake
  public static final int kIntakeId = 10;
  public static final int kLightSensorID = 1;

  // Shooter
  public static final int kShooterId = 9;

  // Climber

  public static final int kClimbID = 11;
  public static final int kCLimbSwitchID = 4;

  public static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          Units.inchesToMeters(kFrontLeftXPosInches),
          Units.inchesToMeters(kFrontLeftYPosInches),
          kInvertLeftSide);
  public static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          Units.inchesToMeters(kFrontRightXPosInches),
          Units.inchesToMeters(kFrontRightYPosInches),
          kInvertRightSide);
  public static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          Units.inchesToMeters(kBackLeftXPosInches),
          Units.inchesToMeters(kBackLeftYPosInches),
          kInvertLeftSide);
  public static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          Units.inchesToMeters(kBackRightXPosInches),
          Units.inchesToMeters(kBackRightYPosInches),
          kInvertRightSide);

  public static final CommandSwerveDrivetrain DriveTrain =
      new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
}
