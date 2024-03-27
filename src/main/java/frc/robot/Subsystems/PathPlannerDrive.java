// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.Subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PathPlannerDrive extends SubsystemBase {

  public PathPlannerDrive() {
    // All other subsystem initialization
    // ...

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(

    // <3 These are set up to do the correct things but is set up as Functions
    
            //DriveSubsystem.getPose2d(),  // Robot pose supplier
            //DriveSubsystem.ResetPosition(), // Method to reset odometry (will be called if your auto has a starting pose)
            //DriveSubsystem.getRobotRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            //DriveSubsystem.DriveChassie(), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

    // <3 There are set up to be allowed by the system, but don't work correctly
    
            DriveSubsystem.getPose2dSupplied(), //Pose supplier
            DriveSubsystem.getPose2dConsumer(), // Method to reset odmetry
            DriveSubsystem.getChassisSpeedsSupplier(), // ChassisSpeeds
            DriveSubsystem.getChassisSpeedsConsumer(), // Method that will drive robot with chassis
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
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
