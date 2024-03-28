// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
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
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.Constants.Constants;
import frc.robot.Constants.PathPlannerReqConstants;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class DriveSubsystem extends SubsystemBase {
  AHRS navx = new AHRS();
  public static double MaxSpeed = 5.5; // 6 meters per second desired top speed
  public static double speedValue = MaxSpeed;
  private static double CreepSpeed = 0.8; // creep mode speed
  public static double MaxAngularRate = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final static CommandXboxController joystick = new CommandXboxController(0); // Driver Joystick
  public final static Pigeon2 Gyro = new Pigeon2(25);

  TalonFX FrontLeftSteerMotor = new TalonFX(Constants.kFrontLeftSteerMotorId);
  TalonFX FrontRightSteerMotor = new TalonFX(Constants.kFrontRightSteerMotorId);
  TalonFX BackLeftSteerMotor = new TalonFX(Constants.kBackLeftSteerMotorId);
  TalonFX BackRightSteerMotor = new TalonFX(Constants.kBackRightSteerMotorId);

  TalonFX FrontLeftDriveMotor = new TalonFX(Constants.kFrontLeftDriveMotorId);
  TalonFX FrontRightDriveMotor = new TalonFX(Constants.kFrontRightDriveMotorId);
  TalonFX BackLeftDriveMotor = new TalonFX(Constants.kBackLeftDriveMotorId);
  TalonFX BackRightDriveMotor = new TalonFX(Constants.kBackRightDriveMotorId);

  SwerveDrivePoseEstimator SwerveEstie = new
  SwerveDrivePoseEstimator(PathPlannerReqConstants.swerveKinematics,  getAngleRotation2d(), getStates(), new Pose2d());

  // public static double controlmodeY = -joystick.getLeftY();
  // public static double controlmodeX = -joystick.getLeftX();

  public void periodic() {

  }

  public void ToggleGear() {
    if (speedValue == MaxSpeed) {
      speedValue = CreepSpeed;
      SmartDashboard.putBoolean("High Gear", false);
    } else if (speedValue == CreepSpeed) {
      speedValue = MaxSpeed;
      SmartDashboard.putBoolean("High Gear", true);
    }
  }

  public Command cmdToggleGear() {
    return this.runOnce(this::ToggleGear);
  }

  // public void InvertDrive() {
  // if (controlmodeY == -joystick.getLeftY() || controlmodeX ==
  // -joystick.getLeftX()) {
  // controlmodeY = joystick.getLeftY();
  // controlmodeX = joystick.getLeftX();
  // SmartDashboard.putBoolean("Drive Inverted", false);
  // } else if (controlmodeY == joystick.getLeftY() || controlmodeX ==
  // joystick.getLeftX()) {
  // controlmodeY = -joystick.getLeftY();
  // controlmodeX = -joystick.getLeftX();
  // SmartDashboard.putBoolean("Drive Inverted", true);
  // }
  // }

  // public Command cmdInvertDrive() {
  // return this.runOnce(this::InvertDrive);
  // }

  public static void Pigeon2Reset() {
    Gyro.reset();
  }

  public static Rotation2d getAngleRotation2d() {

    Rotation2d angle = Gyro.getRotation2d();
    return angle;
  }

  public void ResetPosition() {
    ResetPosition(new Pose2d());
  }

  public void ResetPosition(Pose2d newPose){
    this.
    SwerveEstie.resetPosition(getAngleRotation2d(), getStates(), newPose);

  }

  public Pose2d getPose(){
    return this.SwerveEstie.getEstimatedPosition();
  }

  public static SwerveModuleState getFrontLeftState() {
    TalonFX DriveMotor = new TalonFX(Constants.kFrontLeftDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kFrontLeftSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  public static SwerveModuleState getFrontRightState() {
    TalonFX DriveMotor = new TalonFX(Constants.kFrontRightDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kFrontRightSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  public static SwerveModuleState getBackLeftState() {
    TalonFX DriveMotor = new TalonFX(Constants.kBackLeftDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kBackLeftSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  public static SwerveModuleState getBackRightState() {
    TalonFX DriveMotor = new TalonFX(Constants.kBackRightDriveMotorId);
    TalonFX SteerMotor = new TalonFX(Constants.kBackRightSteerMotorId);
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DriveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
  }

  public static ChassisSpeeds getRobotRelativeSpeeds() {
    return PathPlannerReqConstants.swerveKinematics.toChassisSpeeds(getFrontLeftState(),
        getFrontRightState(),
        getBackLeftState(),
        getBackRightState());
  }

  public static double MPSToRPS(double wheelMPS, double circumference) {
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / MaxSpeed;
      FrontLeftDriveMotor.setControl(driveDutyCycle);
      FrontRightDriveMotor.setControl(driveDutyCycle);
      BackLeftDriveMotor.setControl(driveDutyCycle);
      BackRightDriveMotor.setControl(driveDutyCycle);

    } else {
      driveVelocity.Velocity = MPSToRPS(desiredState.speedMetersPerSecond, Constants.kwheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      FrontLeftDriveMotor.setControl(driveVelocity);
      FrontRightDriveMotor.setControl(driveVelocity);
      BackLeftDriveMotor.setControl(driveVelocity);
      BackRightDriveMotor.setControl(driveVelocity);
    }
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    final PositionVoltage anglePosition = new PositionVoltage(0);

    desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(Gyro.getAngle()));
    FrontLeftSteerMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    FrontRightSteerMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    BackLeftSteerMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    BackRightSteerMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));

    setSpeed(desiredState, isOpenLoop);
  }

  public SwerveModulePosition[] getStates() {
    SwerveModulePosition frontLeft = new SwerveModulePosition(FrontLeftDriveMotor.getPosition().getValue(), getFrontLeftState().angle);
    SwerveModulePosition frontRight = new SwerveModulePosition(FrontRightDriveMotor.getPosition().getValue(), getFrontRightState().angle);
    SwerveModulePosition rearLeft = new SwerveModulePosition(BackLeftDriveMotor.getPosition().getValue(), getBackLeftState().angle);
    SwerveModulePosition rearRight = new SwerveModulePosition(BackRightDriveMotor.getPosition().getValue(), getBackRightState().angle);

    SwerveModulePosition[] states = new SwerveModulePosition[] {frontLeft, frontRight, rearLeft, rearRight};
    return states;
  }

  // public void setModuleStates(SwerveModuleState[] desiredStates) {
  // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MaxSpeed);
  // }

  // public void setModuleStatesB(SwerveModuleState desiredStates) {
  // setDesiredState(desiredStates, false);
  // }

  // public void driveRobotRelative(ChassisSpeeds speeds) {
  // SwerveModuleState[] states =
  // PathPlannerReqConstants.swerveKinematics.toSwerveModuleStates(speeds);
  // SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveSubsystem.MaxSpeed);
  // setModuleStates(states);
  // setDesiredState(states, false);
  // }

  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MaxSpeed);
    SwerveModuleState FrontLeftState = desiredStates[0];
    SwerveModuleState FrontRightState = desiredStates[1];
    SwerveModuleState RearLeftState = desiredStates[2];
    SwerveModuleState RearRightState = desiredStates[3];

    Constants.DriveTrain.getModule(0).apply(FrontLeftState, DriveRequestType.OpenLoopVoltage);
    Constants.DriveTrain.getModule(1).apply(FrontRightState, DriveRequestType.OpenLoopVoltage);
    Constants.DriveTrain.getModule(2).apply(RearLeftState, DriveRequestType.OpenLoopVoltage);
    Constants.DriveTrain.getModule(3).apply(RearRightState, DriveRequestType.OpenLoopVoltage);

  }

  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = PathPlannerReqConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Returns the current heading of the robot.
   *
   * @return value from 0 to 180 degrees.
   */
  public static double getAngleP() {
    double angle = Gyro.getYaw().getValue();
    SmartDashboard.putNumber("GyroP", angle);
    angle = Math.toRadians(angle);
    return angle;
  }

  // public static void DriveChassie() {

  // ChassisSpeeds speeds = getRobotRelativeSpeeds();
  // drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond);
  // }

  public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  public static double swerveX(CommandXboxController Stick) {
    double sX;
    sX = -(Stick.getLeftX() * Math.cos(getAngleP()) - (Stick.getLeftY() * Math.sin(getAngleP())));
    // sX = Stick.getLeftX();
    return -sX;
  }

  public static double swerveY(CommandXboxController Stick) {
    double sY;
    sY = (Stick.getLeftY() * Math.cos(getAngleP()) + (Stick.getLeftX() * Math.sin(getAngleP())));
    // sY = Stick.getLeftY();
    return sY;
  }

  public static double swerveZ(CommandXboxController Stick) {
    double sZ;
    sZ = Stick.getRightX();
    return sZ;
  }

  public static CommandXboxController getJoystick() {
    return joystick;
  }

  public DriveSubsystem() {

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        resetPose -> this.ResetPosition(resetPose), // Method to reset odometry (will be called if your auto has a starting pose)
        () -> {return this.getRobotRelativeSpeeds();}, // ChassisSpeeds Òupplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }
}