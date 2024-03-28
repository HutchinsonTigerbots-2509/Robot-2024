// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.PresetPos.MainPos;
import frc.robot.Subsystems.Door;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PotatoShoot extends InstantCommand {

  private Command commandSequence;

  Shooter shooter;
  Door door;
  Intake intake;
  DriveSubsystem drivetrain;

  public PotatoShoot(DriveSubsystem pDrivetrain, Intake pIntake, Door pDoor, Shooter pShooter) {

    drivetrain = pDrivetrain;
    intake = pIntake;
    door = pDoor;
    shooter = pShooter;

    commandSequence =
        Commands.sequence(
            new MainPos(shooter, door).withTimeout(3),
            shooter
                .cmdShoot()
                .withTimeout(2)
                .andThen(intake.cmdIn().alongWith(shooter.cmdShoot()).withTimeout(2)),
            shooter.cmdStop());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandSequence.schedule();
  }
}
