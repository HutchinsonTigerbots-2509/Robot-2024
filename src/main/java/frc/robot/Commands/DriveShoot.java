// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Drivetrain.a;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Constants.PathPlannerReqConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveShoot extends SequentialCommandGroup {
  /** Creates a new DriveShoot. */
  public DriveShoot(Shooter pShooter, Intake pIntake, DriveSubsystem pDrive) {
    Shooter sShooter = pShooter;
    Intake sIntake = pIntake;
    DriveSubsystem sDrive = pDrive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Shoot(sShooter).withTimeout(1),
      new IntakeIn(sIntake).withTimeout(1),
      new a(sDrive, 1, 0)
    );
  }
}
