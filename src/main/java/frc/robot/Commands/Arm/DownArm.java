// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Door;

public class DownArm extends Command {
  /** Creates a new GrabClose. */
  private Door door;

  public DownArm(Door pDoor) {

    door = pDoor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(door);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    door.MoveDoor(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    door.DoorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}