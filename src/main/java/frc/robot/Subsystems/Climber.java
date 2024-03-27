// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  public TalonSRX Climber = new TalonSRX(Constants.kClimbID);

  public Climber() {
    Climber.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Moves climber up to grab chain */
  public void climbExtend() {
    Climber.set(ControlMode.PercentOutput, .3);
  }

  /** Pulls up robot by bringing climber down */
  public void climbRetract() {
      Climber.set(ControlMode.PercentOutput, -.3);
  }

  /** Turns off climber */
  public void climbStop() {
    Climber.set(ControlMode.PercentOutput, 0);
  }
}
