// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkMaxLimitSwitch;

public class ShoulderSubsystem extends SubsystemBase {
  /** Creates a new ShoulderSubsystem. */
  CANSparkMax shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushed);

  private static SparkMaxLimitSwitch frontShoulderLimitSwitch;
  private static SparkMaxLimitSwitch rearShoulderLimitSwitch;

  public boolean protectionEnabled = true;
  public boolean maxedOut = false;

  static RelativeEncoder shoulderEncoderRelative1;
//5 MAX, 4.22 START, 0 MIN
  public static void resetShoulderEncoder() {
    shoulderEncoderRelative1.setPosition(4.22);
  }

  public ShoulderSubsystem() {
   
    //shoulderEncoderAnalog = shoulder.get(Mode.kRelative);
    shoulderEncoderRelative1 = shoulder.getEncoder(Type.kQuadrature, 4096);
    resetShoulderEncoder();
    //shoulderEncoderRelative1 = shoulder.getAlternateEncoder(Type.kQuadrature, 4);
   // frontShoulderLimitSwitch = shoulder.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
   // rearShoulderLimitSwitch = shoulder.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }
/* 
  public boolean getFrontShoulderLimitSwitch(){
    return false;//rontShoulderLimitSwitch.isPressed();
  }

  public boolean getRearShoulderLimitSwitch(){
    return false;//rearShoulderLimitSwitch.isPressed();
  }*/

  public void move(double speed){
    /*if(!getFrontShoulderLimitSwitch() && !getRearShoulderLimitSwitch() && Math.abs(speed) >= 0.1){
      //Move if limit switches are false
      shoulder.set(speed);
    }
    else if(getFrontShoulderLimitSwitch() && speed <= -0.1){
        //Dont move further than front switch
        shoulder.set(speed);
      }
    else if(getRearShoulderLimitSwitch() && speed >= 0.1){
      //Dont move further than rear switch
      shoulder.set(speed);
    }
    else{
      //Stop
      shoulder.set(speed);
    }*/
    maxedOut = false;
     if (protectionEnabled) {
      if (shoulderEncoderRelative1.getPosition() >= 5){
        if (speed <= -0.1){
          shoulder.set(speed);
        }
        else{
          maxedOut = true;
          shoulder.set(0.0);
        }
      }
  
      else if (shoulderEncoderRelative1.getPosition() < 0.05) {
        if (speed >= 0.1){
          shoulder.set(speed);
        }
        else{
          maxedOut = true;
          shoulder.set(0.0);
        }
      }
      else{
        shoulder.set(speed);
      }
      
    }
   else {
      shoulder.set(speed);
    }

    SmartDashboard.putBoolean("maxed", maxedOut);
   

  }

   /*  public static boolean isShoulderBack(){
      return rearShoulderLim
      itSwitch.isPressed();
    } */

    public static double getEncoderTicks(){
      return 0.0;//shoulderEncoder.getPosition();
    }


  @Override
  public void periodic() {
    
    SmartDashboard.putBoolean("shoulderProtected", protectionEnabled);
   // if (shoulderEncoderAnalog != null) SmartDashboard.putNumber("Shoulder encoder analog", shoulderEncoderAnalog.getPosition());
    if (shoulderEncoderRelative1 != null) SmartDashboard.putNumber("Shoulder encoder", shoulderEncoderRelative1.getPosition());
 //   if (shoulderEncoderRelative2 != null) SmartDashboard.putNumber("Shoulder encoder relative 2", shoulderEncoderRelative2.getPosition());
   // SmartDashboard.putNumber("Shoulder current", shoulder.getOutputCurrent());
    

    
  }
}
