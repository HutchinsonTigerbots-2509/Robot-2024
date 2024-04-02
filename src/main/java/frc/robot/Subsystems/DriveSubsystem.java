// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.Commands.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Constants.Constants;
import frc.robot.Constants.PathPlannerReqConstants;

public class DriveSubsystem extends SubsystemBase {
  public static double MaxSpeed = 5.5; // 6 meters per second desired top speed
  public static double speedValue = MaxSpeed;
  private static double CreepSpeed = 0.8; // creep mode speed
  public static double MaxAngularRate = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  public final static Pigeon2 Gyro = new Pigeon2(Constants.kPigeonID);
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain

  final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  final Telemetry logger = new Telemetry(MaxSpeed);

  TalonFX FrontLeftSteerMotor = new TalonFX(Constants.kFrontLeftSteerMotorId);    
  TalonFX FrontRightSteerMotor = new TalonFX(Constants.kFrontRightSteerMotorId);    
  TalonFX BackLeftSteerMotor = new TalonFX(Constants.kBackLeftSteerMotorId);
  TalonFX BackRightSteerMotor = new TalonFX(Constants.kBackRightSteerMotorId);

  TalonFX FrontLeftDriveMotor = new TalonFX(Constants.kFrontLeftDriveMotorId);    
  TalonFX FrontRightDriveMotor = new TalonFX(Constants.kFrontRightDriveMotorId);    
  TalonFX BackLeftDriveMotor = new TalonFX(Constants.kBackLeftDriveMotorId);
  TalonFX BackRightDriveMotor = new TalonFX(Constants.kBackRightDriveMotorId);
  Pose2d StartingPose = new Pose2d(Translation2d(0, 0), getAngleRotation2d());

  SwerveDrivePoseEstimator SwerveEstie = new SwerveDrivePoseEstimator(PathPlannerReqConstants.swerveKinematics, getAngleRotation2d(), getStates(), StartingPose);
    


  public void periodic() {}

  /** Switches our Speed for a creep and normal mode driving */
  public void ToggleGear() { 
    if (speedValue == MaxSpeed) {
      speedValue = CreepSpeed;
      SmartDashboard.putBoolean("High Gear", false);
    } else if (speedValue == CreepSpeed) {
      speedValue = MaxSpeed;
      SmartDashboard.putBoolean("High Gear", true);
    }
  }

  /** The command to call the toggle gear function */
  public Command cmdToggleGear() {
    return this.runOnce(this::ToggleGear);
  }

  /** Resets our pigeon 2 gyro */
  public void Pigeon2Reset() {
    Gyro.reset();
  }

  public static double MPSToRPS(double wheelMPS, double circumference){
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  /** Returns the current angle of our robot using the pigeon 2 gyro */
  public static double getAngleP() {
    double angle = Gyro.getYaw().getValue();
    SmartDashboard.putNumber("GyroP", angle);
    angle = Math.toRadians(angle);
    return angle;
  }


  /** Our swerve drive object we call to give our speeds */
  public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Converts the controller to the robots X-axis for driving with field oriented */
  public static double swerveX(CommandXboxController Stick) {
    double sX;
    sX = -(Stick.getLeftX() * Math.cos(getAngleP()) - (Stick.getLeftY() * Math.sin(getAngleP())));
    //sX = Stick.getLeftX();
    return -sX;
  }

  /** Converts the controller to the robots Y-axis for driving with field oriented */
  public static double swerveY(CommandXboxController Stick) {
    double sY;
    sY = (Stick.getLeftY() * Math.cos(getAngleP()) + (Stick.getLeftX() * Math.sin(getAngleP())));
    //sY = Stick.getLeftY();
    return sY;
  }

  /** Converts the controller to the robots Z-axis for driving with field oriented */
  public static double swerveZ(CommandXboxController Stick) {
    double sZ;
    sZ = Stick.getRightX();
    return sZ;
  }

  /** Gets the Rotation2d from the pigeon */
  public Rotation2d getAngleRotation2d() {
    Rotation2d angle = Gyro.getRotation2d();
    return angle;
  }

  /** Gets our Pose2d by using our Translation2d and our Pose2d*/
  public Pose2d getPose2d() {
    //Pose2d pose = new Pose2d(getTranslation2d(), getAngleRotation2d());
    Pose2d pose = SwerveEstie.getEstimatedPosition();
    return pose;
  }

  /** Turns a given x and y into a Translation2d */
  private static Translation2d Translation2d(int x, int y) {
    Translation2d trans = new Translation2d(x, y);
    return trans;
  }

  /** Resets the current Pose2d of the robot */
  public void ResetPosition(Pose2d pos) {
     SwerveEstie.resetPosition(getAngleRotation2d(), getStates(), pos);
   }

  /** Drives the robot by converting ChassisSpeeds back to just our field oriented X,Y,Z cords */
  public void DriveChassie(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("MetersPerSecondX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("MetersPerSecondY", speeds.vyMetersPerSecond);
    //drivetrain.applyRequest(() -> DriveSubsystem.drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    //DriveSubsystem.drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond);
    drivetrain.setControl(DriveSubsystem.drive.withVelocityX(-speeds.vxMetersPerSecond).withVelocityY(-speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  /** Gets the current swerveModule States */
  public SwerveModulePosition[] getStates() {

    SwerveModulePosition frontLeft = new SwerveModulePosition(FrontLeftDriveMotor.getPosition().getValue(), getAngleRotation2d());
    SwerveModulePosition frontRight = new SwerveModulePosition(FrontRightDriveMotor.getPosition().getValue(), getAngleRotation2d());
    SwerveModulePosition rearLeft = new SwerveModulePosition(BackLeftDriveMotor.getPosition().getValue(), getAngleRotation2d());
    SwerveModulePosition rearRight = new SwerveModulePosition(BackRightDriveMotor.getPosition().getValue(), getAngleRotation2d());
    
    SwerveModulePosition[] states = new SwerveModulePosition[] {
      frontLeft,
      frontRight,
      rearLeft,
      rearRight
    };
    return states;
  }

  /** Converts the states of the different modules of the robot into a ChassisSpeed */
  public ChassisSpeeds getRobotRelativeSpeeds(){
      return PathPlannerReqConstants.swerveKinematics.toChassisSpeeds(getFrontLeftState(),
                                                           getFrontRightState(),
                                                           getBackLeftState(),
                                                           getBackRightState());
  }

  /** Gets the SwerveModuleState of the FrontLeftModule in our swerve drive */
  public static SwerveModuleState getFrontLeftState() {
    TalonFX DriveMotor = new TalonFX(Constants.kFrontLeftDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kFrontLeftSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  /** Gets the SwerveModuleState of the FrontRightModule in our swerve drive */
  public static SwerveModuleState getFrontRightState() {
    TalonFX DriveMotor = new TalonFX(Constants.kFrontRightDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kFrontRightSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  /** Gets the SwerveModuleState of the RearLeftModule in our swerve drive */
  public static SwerveModuleState getBackLeftState() {
    TalonFX DriveMotor = new TalonFX(Constants.kBackLeftDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kBackLeftSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  /** Gets the SwerveModuleState of the RearRightModule in our swerve drive */
  public static SwerveModuleState getBackRightState() {
    TalonFX DriveMotor = new TalonFX(Constants.kBackRightDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kBackRightSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  public void DriveRoboRel(double x, double y) {
    drivetrain.setControl(drive.withVelocityX(x * MaxSpeed).withVelocityY(y * MaxSpeed));
  }
  
}