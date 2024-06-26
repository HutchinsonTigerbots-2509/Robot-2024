// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PathPlannerDrive extends SubsystemBase {

  public PathPlannerDrive(DriveSubsystem drive) {
    // All other subsystem initialization
    // ...

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
    
            drive::getPose2d,  // Robot pose supplier
            resetPose -> drive.ResetPosition(resetPose), // Method to reset odmetry
            drive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            robotRelativeOutput -> drive.DriveChassie(robotRelativeOutput), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(.07, 0.000001, 0), // Translation PID constants
                    new PIDConstants(.01, 0.000001, 0), // Rotation PID constants
                    5.5, // Max module speed, in m/s
                    .3302, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
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
            drive // Reference to this subsystem to set requirements
    );
  }
}
