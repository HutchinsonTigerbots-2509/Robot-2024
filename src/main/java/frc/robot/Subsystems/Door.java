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
  public DutyCycleEncoder DoorEncoder = new DutyCycleEncoder(0);

  public DigitalInput TopLimitSwitch = new DigitalInput(Constants.kTopLimitSwitchID);
  public DigitalInput BottomLimitSwitch = new DigitalInput(Constants.kBottomLimitSwitchID);

  public Door() {
    DoorEncoder.setPositionOffset(88);
    DoorEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!TopLimitSwitch.get()) {
      DoorEncoder.reset();
    }
  }

  /** Moves the door up off from the ground */
  public void DoorOpen(double Speed) {
    Door.set(Speed);
  }

  /** Moves the door down onto the ground */
  public void DoorClose(double Speed) {
    Door.set(-Speed);
  }

  /** Used to move the door up and down based on it hitting the top and bottom limit switches using the input speed*/
  public void MoveDoor(double Speed) {
    if (Speed > 0 && !TopLimitSwitch.get()){
      Door.set(0);
    }
    else if (Speed < 0 && !BottomLimitSwitch.get()) {
      Door.set(0);
    }
    else{
      Door.set(Speed);
    }
  }

  /** Turns off the door */
  public void DoorStop() {
    Door.set(0);
  }

  /** Returns the current position of the door as a double */
  public double getAngle(){
    return DoorEncoder.get()*360;
  }
  
  /** Returns the top limit switch as a boolean */
  public Boolean getTopLimit() {
    boolean top = !TopLimitSwitch.get();
    return top;
  }

  /** Returns the bottom limit switch as a boolean */
  public Boolean getbottomLimit() {
    boolean bottom = !BottomLimitSwitch.get();
    return bottom;
  }
}
