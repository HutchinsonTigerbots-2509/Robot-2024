// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PresetPos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Door.TeleDoorController;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Commands.Shooter.Stop;
import frc.robot.Subsystems.Door;
import frc.robot.Subsystems.Shooter;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFarPos extends SequentialCommandGroup {

    private Shooter shooter;
    private Door door;

  public ShootFarPos(Shooter pShooter, Door pDoor) {
    this.shooter = pShooter;
    this.door = pDoor;

    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        Commands.parallel(
           new TeleDoorController(99.5, door),
           new Shoot(shooter)));
  }
}
