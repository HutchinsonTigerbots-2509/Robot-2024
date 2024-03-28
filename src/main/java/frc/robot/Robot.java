// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.getDrivetrain().Pigeon2Reset();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Door Enc", m_robotContainer.getDoor().getAngle());
    SmartDashboard.putBoolean("Light Sensor", m_robotContainer.getIntake().getLightSensor());
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
    // Drive forward with negative Y (forward)
    // DriveSubsystem.drive.withVelocityX(DriveSubsystem.getX(false, 0))
    //         .withVelocityY(DriveSubsystem.getY(false, 0)) // Drive left with negative X (left)
    //         .withRotationalRate(DriveSubsystem.getZ(false, 0));
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Drive forward with negative Y (forward)
    // DriveSubsystem.drive.withVelocityX(DriveSubsystem.getX(false, 0))
    //         .withVelocityY(DriveSubsystem.getX(false, 0)) // Drive left with negative X (left)
    //         .withRotationalRate(DriveSubsystem.getZ(false, 0));
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
