// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PresetPos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Door.TeleDoorController;
import frc.robot.Subsystems.Door;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WallPosition extends SequentialCommandGroup {

    private Door door;

  public WallPosition(Door pDoor) {
    this.door = pDoor;

    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        //Commands.parallel(
           new TeleDoorController(39, door)
           //new Shoot(shooter))
           );
  }
}
