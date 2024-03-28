// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Shooter;

public class DriveToPosition extends Command {
  DriveSubsystem drive;
  Shooter shooter;
  double X;
  double Y;
  double Z;

  /** Creates a new DriveToPosition. */
  public DriveToPosition(
      DriveSubsystem drivetrain, Shooter pShooter, double pYSpeed, double pXSpeed, double pZSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drivetrain;
    X = pXSpeed;
    Y = pYSpeed;
    Z = pZSpeed;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*  Constants.DriveTrain.applyRequest(() -> */
    DriveSubsystem.drive
        .withVelocityX(X) // Drive forward with negative Y (forward)
        .withVelocityY(Y) // Drive left with negative X (left)
        .withRotationalRate(Z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*  Constants.DriveTrain.applyRequest(() -> */
    DriveSubsystem.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
    // shooter.Shoot(3000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
