// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonSRX Shooter = new TalonSRX(Constants.kShooterId);

  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Shoot(double Velocity) {
    // Shooter.set(ControlMode.Velocity, Velocity);
    Shooter.set(ControlMode.PercentOutput,-1);
  }

  public void Stop() {
    Shooter.set(ControlMode.Velocity, 0);
  }
}
