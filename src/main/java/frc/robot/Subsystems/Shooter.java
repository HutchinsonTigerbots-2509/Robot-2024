// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  static TalonSRX Shooter = new TalonSRX(Constants.kShooterId);

  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Shoot(double Velocity) {
    // Shooter.set(ControlMode.Velocity, Velocity);
    Shooter.set(ControlMode.PercentOutput, -1);
  }

  public void ShootOut(double Velocity) {
    Shooter.set(ControlMode.PercentOutput, 1);
  }

  public void Stop() {
    Shooter.set(ControlMode.Velocity, 0);
  }

  public Command cmdStop() {
    return this.runOnce(this::Stop);
  }

  public Command cmdShoot() {
    return this.runEnd(
        () -> {
          this.Shoot(3000);
        },
        this::Stop);
  }

  public Command cmdShootOut() {
    return this.runEnd(
        () -> {
          this.ShootOut(3000);
        },
        this::Stop);
  }
}
