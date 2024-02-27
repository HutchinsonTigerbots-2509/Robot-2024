// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public TalonSRX Intake = new TalonSRX(Constants.kIntakeId);

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void IntakeIn() {
    Intake.set(ControlMode.PercentOutput, 1);
  }

  public void IntakeOut() {
    Intake.set(ControlMode.PercentOutput, -1);
  }

  public void IntakeStop() {
    Intake.set(ControlMode.PercentOutput, 0);
  }
}
