// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Door extends SubsystemBase {
  /** Creates a new Arm. */
  public TalonFX Door = new TalonFX(Constants.kArmId);
  public DutyCycleEncoder natecoder = new DutyCycleEncoder(0);
  public double desiredPos = 0;

  

  public DigitalInput TopLimitSwitch = new DigitalInput(Constants.kTopLimitSwitchID);
  public DigitalInput BottomLimitSwitch = new DigitalInput(Constants.kBottomLimitSwitchID);

  public Door() {

    
    natecoder.setPositionOffset(88);
    natecoder.reset();
    // enc.setDistancePerRotation(1024);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Limit Switch", !TopLimitSwitch.get());
    SmartDashboard.putBoolean("Bottom Limit Switch", !BottomLimitSwitch.get());
    SmartDashboard.putNumber("natecoder", getAngle());
    // This method will be called once per scheduler run
    if (!TopLimitSwitch.get()) {
      natecoder.reset();
    }

    // if (!BottomLimitSwitch.get()) {
    //   natecoder.reset();
    //   natecoder.setPositionOffset(122);
    // }
  }

  public void DoorOpen(double Speed) {
      Door.set(Speed);
      SmartDashboard.putNumber("SpeedUp", Speed);
  }

  public void DoorClose(double Speed) {
      Door.set(-Speed);
      SmartDashboard.putNumber("SpeedDown", Speed);
  }

  public void DoorPosUp() {
    if (desiredPos < 200) {
      desiredPos += .1;
    }
  }

  // Command to move the shoulder forward function
  public Command cmdDoorPosUp() {
    return this.run(this::DoorPosUp);
  }

  public void DoorPosDown() {
    if (desiredPos > -200) {
      desiredPos -= .1;
    }
  }

  // Command to move the shoulder backward function
  public Command cmdDoorPosDOwn() {
    return this.run(this::DoorPosDown);
  }

  public void MoveDoor(double Speed) {
    if (Speed > 0 && !TopLimitSwitch.get()){
      Door.set(0);
    }
    else if (Speed < 0 && !BottomLimitSwitch.get()) {
      Door.set(0);
    }
    else{
      Door.set(Speed);
      SmartDashboard.putNumber("Speed", Speed);
    }
  }

  public void DoorStop() {
    Door.set(0);
  }

  public double getAngle(){
    return natecoder.get()*360;
  }

  // public double getAngle2(){
  //   return Door.getselectedsensor
  // }

  public double getDesiredPos() {
    return desiredPos;
  }
}
