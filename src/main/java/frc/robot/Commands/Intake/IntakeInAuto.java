// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeInAuto extends Command {
  /** Creates a new GrabClose. */
  private Intake intake;

  public IntakeInAuto(Intake pIntake) {

    intake = pIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.IntakeInAuto();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.IntakeInAuto();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.IntakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}