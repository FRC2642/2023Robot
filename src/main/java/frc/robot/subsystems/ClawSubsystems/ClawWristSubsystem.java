// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClawWristSubsystem extends SubsystemBase {

  public CANSparkMax wrist = new CANSparkMax(24, MotorType.kBrushed);
  public static SparkMaxAnalogSensor wristEncoder;
  public static SparkMaxLimitSwitch wristLimitSwitch;
  private PIDController wristPID = new PIDController(0.2, 0, 0);

  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    wristEncoder = wrist.getAnalog(Mode.kRelative);
    wristLimitSwitch = wrist.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist encoder", getEncoderTicks());
  }

 

  public static double getEncoderTicks() {
    return wristEncoder.getPosition();
  }
  public void stop() {
    wrist.set(0);
  }
  public void move(double speed) {
    // If limit switch is hit, prevents wrist from moving wrist in the same direction further
    //if (wrist.getLimitSwitchState() == false) {
      // prevents wrist from twisting inside of robot
      /*if (!clawInRobot()){
        if (ClawWristSubsystem.getEncoderTicks() > -180 && speed <= -0.1) {
          wrist.set(speed);
        }
        else if (ClawWristSubsystem.getEncoderTicks() < 180 && speed >= 0.1) {
          wrist.set(speed);
        }
        else {
          stop();
        }
      }
      else{
        stop();
      }*/
      wrist.set(speed);
    //} 
  }
  /*public boolean getLimitSwitchState() {
    return wristLimitSwitch.isPressed();
  }

  public void flipClaw(){
    //Uh oh button
    if (Math.abs(getEncoderTicks()) > 100 && !clawInRobot()){
      move(wristPID.calculate(getEncoderTicks(), 0));
    }
    else if (Math.abs(getEncoderTicks()) < 25 && !clawInRobot()){
      move(wristPID.calculate(getEncoderTicks(), 180));
    }
  }

  public void setWrist(double speed){
    if (speed == -1){
      move(wristPID.calculate(getEncoderTicks(), -90));
    } else if (speed <= -.2){
      move(wristPID.calculate(getEncoderTicks(), -45));
    } else if (speed < .2){
      move(wristPID.calculate(getEncoderTicks(), 0));
    } else if (speed < 1){
      move(wristPID.calculate(getEncoderTicks(), 45));
    } else{
      move(wristPID.calculate(getEncoderTicks(), 90));
    }
  }*/

  /*public boolean clawInRobot(){
    //checks if claw is inside sliders /*no number are correct
    return ((ShoulderSubsystem.getEncoderTicks() < 10 && ((CarriageSubsystem.getCarriageEncoder() > 90 ) || (!SliderSubsystem.isSliderBack()))) || 
    (ShoulderSubsystem.getEncoderTicks() > 170 && ((CarriageSubsystem.getCarriageEncoder() < 90) || SliderSubsystem.isSliderBack())));
  }*/

  

}
