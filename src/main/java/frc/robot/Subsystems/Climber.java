// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public TalonSRX Climber = new TalonSRX(Constants.kClimbID);

  // public DigitalInput ClimbSwitch = new DigitalInput(Constants.kCLimbSwitchID);

  public Climber() {
    Climber.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbExtend() {
    Climber.set(ControlMode.PercentOutput, .3);
  }

  public void climbRetract() {
    Climber.set(ControlMode.PercentOutput, -.3);
  }

  public void climbStop() {
    Climber.set(ControlMode.PercentOutput, 0);
  }

  public Command cmdExtend() {
    return this.runEnd(this::climbExtend, this::climbStop);
  }

  public Command cmdRetract() {
    return this.runEnd(this::climbRetract, this::climbStop);
  }
}
