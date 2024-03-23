// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public TalonSRX Intake = new TalonSRX(Constants.kIntakeId);

  public DigitalInput LightSensor = new DigitalInput(Constants.kLightSensorID);
  

  public Intake() {
    // LightSensor.resetAccumulator();
    SmartDashboard.putBoolean("Proximity Sensor", getLightSensor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void IntakeIn() {
    if (Shooter.Shooter.getMotorOutputPercent() < -0.1) {
      Intake.set(ControlMode.PercentOutput, 1);
    }
    else if (LightSensor.get()) {
    Intake.set(ControlMode.PercentOutput, 0);
    } else if (!LightSensor.get()) {
      Intake.set(ControlMode.PercentOutput, 1);
    }
  }

  public void IntakeOut() {
    if (Shooter.Shooter.getMotorOutputPercent() < -0.1) {
      Intake.set(ControlMode.PercentOutput, -1);
    } else if (LightSensor.get()) {
    Intake.set(ControlMode.PercentOutput, -1);
    } else if (!LightSensor.get()) {
      Intake.set(ControlMode.PercentOutput, -1);
    }
  }

  public void IntakeStop() {
    Intake.set(ControlMode.PercentOutput, 0);
  }

  public boolean getLightSensor() {
    return LightSensor.get();
  }
}
