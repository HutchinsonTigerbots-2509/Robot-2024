// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private DriveSubsystem m_DriveSubsystem;

  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain

  final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  final Telemetry logger = new Telemetry(m_DriveSubsystem.MaxSpeed);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.getDrivetrain().Pigeon2Reset();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> DriveSubsystem.drive.withVelocityX(DriveSubsystem.swerveY(m_robotContainer.joystick) * DriveSubsystem.speedValue) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(DriveSubsystem.swerveX(m_robotContainer.joystick) * DriveSubsystem.speedValue) // Drive left with negative X (left)
            .withRotationalRate(DriveSubsystem.swerveZ(m_robotContainer.joystick) * DriveSubsystem.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
