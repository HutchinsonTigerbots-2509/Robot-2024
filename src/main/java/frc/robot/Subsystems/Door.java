// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Door extends SubsystemBase {
  /** Creates a new Arm. */
  public TalonFX Door = new TalonFX(Constants.kArmId);
  public DutyCycleEncoder enc = new DutyCycleEncoder(0);

  public Door() {
    enc.setPositionOffset(88);
    enc.reset();
    // enc.setDistancePerRotation(1024);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void DoorOpen(double Speed) {
    Door.set(Speed);
  }

  public void DoorClose(double Speed) {
    Door.set(-Speed);
  }

  public void DoorStop() {
    Door.set(0);
  }

  public double getAngle(){
    return enc.get()*360;
  }
}
