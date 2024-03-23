// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

public class DriveSubsystem extends SubsystemBase {
  AHRS navx = new AHRS();
  public static double MaxSpeed = 5; // 6 meters per second desired top speed
  public static double speedValue = MaxSpeed;
  private static double CreepSpeed = 0.6; // creep mode speed
  public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

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

  public void ResetNavx() {
    navx.reset();
  }

  public double getAngle() {
    double angle = navx.getAngle();
    return angle;
  }

   public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop



  // public DriveSubsystem() {


  //   // Configure AutoBuilder last
  //   AutoBuilder.configureHolonomic(
  //           this::getPose, // Robot pose supplier
  //           this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
  //           this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //           this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  //           new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //                   new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  //                   new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
  //                   4.5, // Max module speed, in m/s
  //                   0.4, // Drive base radius in meters. Distance from robot center to furthest module.
  //                   new ReplanningConfig() // Default path replanning config. See the API for the options here
  //           ),
  //           () -> {
  //             // Boolean supplier that controls when the path will be mirrored for the red alliance
  //             // This will flip the path being followed to the red side of the field.
  //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //             var alliance = DriverStation.getAlliance();
  //             if (alliance.isPresent()) {
  //               return alliance.get() == DriverStation.Alliance.Red;
  //             }
  //             return false;
  //           },
  //           this // Reference to this subsystem to set requirements
  //   );
  // }
}