// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class PathPlannerReqConstants {

  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(0.3048, 0.3048),
          new Translation2d(0.3048, -0.3048),
          new Translation2d(-0.3048, 0.3048),
          new Translation2d(-0.3048, -0.3048));
}
