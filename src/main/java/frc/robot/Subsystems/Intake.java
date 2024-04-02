// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public TalonSRX Intake = new TalonSRX(Constants.kIntakeId);

  public DigitalInput LightSensor = new DigitalInput(Constants.kLightSensorID);

  public boolean proxSensorBypass = false;
  

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
SmartDashboard.putBoolean("HOLDING RING", LightSensor.get());
  }

  public void BypassProxSens() { 
    if (proxSensorBypass == false) {
      proxSensorBypass = true;
      SmartDashboard.putBoolean("PROXIMITY SENSOR BYPASS", true);
    } else if (proxSensorBypass == true) {
      proxSensorBypass = false;
      SmartDashboard.putBoolean("PROXIMITY SENSOR BYPASS", false);
    }
  }

    public Command cmdBypassProxSens() {
    return this.runOnce(this::BypassProxSens);
  }

  /** Intakes rings into the robot unless light sensor is hit */
  public void IntakeIn() {
    if (Shooter.Shooter.getMotorOutputPercent() < -0.1) {
      Intake.set(ControlMode.PercentOutput, 1);
    } else if (proxSensorBypass == true) {
    Intake.set(ControlMode.PercentOutput, 1);
    } else if (LightSensor.get()) {
    Intake.set(ControlMode.PercentOutput, 0);
    } else if (!LightSensor.get()) {
      Intake.set(ControlMode.PercentOutput, 1);
    }
  }

  /** Reverses intake in case of emergencies */
  public void IntakeOut() {
    if (Shooter.Shooter.getMotorOutputPercent() < -0.1) {
      Intake.set(ControlMode.PercentOutput, -1);
    } else if (proxSensorBypass == true) {
    Intake.set(ControlMode.PercentOutput, -1);
    } else if (LightSensor.get()) {
    Intake.set(ControlMode.PercentOutput, -1);
    } else if (!LightSensor.get()) {
      Intake.set(ControlMode.PercentOutput, -1);
    }
  }

  /** Turns intake off */
  public void IntakeStop() {
    Intake.set(ControlMode.PercentOutput, 0);
  }

  /** Grabs Boolean true if a ring is inside the robot */
  public boolean getLightSensor() {
    return LightSensor.get();
  }
}
