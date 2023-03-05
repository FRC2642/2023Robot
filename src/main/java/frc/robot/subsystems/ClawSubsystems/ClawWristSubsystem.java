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
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClawWristSubsystem extends SubsystemBase {

  public CANSparkMax wrist = new CANSparkMax(24, MotorType.kBrushed);
  public RelativeEncoder wristEncoder = wrist.getEncoder(Type.kQuadrature, 4);
  //public static SparkMaxLimitSwitch wristLimitSwitch;
  private PIDController wristPID = new PIDController(0.2, 0, 0);
  public static double DEGREES_PER_TICK = 180d/22d;

  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    wristEncoder.setPositionConversionFactor(DEGREES_PER_TICK);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist encoder", getEncoderTicks());
    //SmartDashboard.putNumber("wrist encoder deg", getEncoderTicks() * DEGREES_PER_TICK);

  }

 

  public double getEncoderTicks() {
    return wristEncoder.getPosition();
  }
  public void stop() {
    wrist.set(0);
  }
  public void resetWristEncoder() {
    wristEncoder.setPosition(180.0);

  }
  public void move(double speed) {
    
      
  /*   if (getEncoderTicks() >= -180) {
      if (speed <= -0.1){
        wrist.set(speed);
      }
      else{
        wrist.set(0.0);
      }
    }
    else if (getEncoderTicks() < 180) {
      if (speed >= 0.1){
        wrist.set(speed);
      }
      else{
        wrist.set(0.0);
      }
    }
    else {*/
      wrist.set(speed);
 //   }
  }
 

  /*public void flipClaw(){
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

  

}
