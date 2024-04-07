// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Door;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.Door;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleDoorController extends PIDCommand {
  static final double kP = 0.15;
  // 0.04 before
  static final double kI = 0.01;
  // .003 before
  static final double kD = 0.00;
  /** Creates a new Shoulder. */
  public TeleDoorController(double PreferredAngle, Door door) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        door::getAngle,
        // This should return the setpoint (can also be a constant)
        PreferredAngle,
        // This uses the output
        output -> {
          // Use the output here
          door.MoveDoor(-output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(door);
    // Configure additional PID options by calling `getController` here.
    this.getController().setTolerance(2);
    this.getController().setSetpoint(PreferredAngle);
  }

  // Returns true when the command shoutld end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
