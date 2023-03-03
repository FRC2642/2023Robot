// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.utils.MathR;

import com.revrobotics.SparkMaxLimitSwitch;


public class SliderSubsystem extends SubsystemBase {
  /** Creates a new SliderSubsystem. */
  
  //Unsure whether or not the motor is inverted, make changes accordingly
  CANSparkMax slider = new CANSparkMax(Constants.MAIN_SLIDER_MOTOR, MotorType.kBrushless);
  RelativeEncoder sliderEncoder = slider.getEncoder();


  public static SparkMaxLimitSwitch frontSliderLimitSwitch;
  public static SparkMaxLimitSwitch rearSliderLimitSwitch;

  private PIDController pid = new PIDController(0.05, 0, 0);
  private static boolean isBack = true;

  public SliderSubsystem() {
    
    //positions.put(SliderPositions.THIRD_POSITION, 10.0);//Right on aux D-pad

    frontSliderLimitSwitch = slider.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rearSliderLimitSwitch = slider.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    sliderEncoder.setPosition(0);
  }

  
  public void move(boolean extend){
    double speed;
    
    
    if (extend){
      speed = MathR.limit(pid.calculate(getSliderEncoderTicks(), -240), -0.9, 0.9);
    }
    else{
      speed = MathR.limit(pid.calculate(getSliderEncoderTicks(), -10), -0.9, 0.9);
    }

    if (getSliderEncoderTicks() <= 11){
      isBack = true;
    }
    else{
      isBack = false;
    }
    slider.set(speed);
  }
  


  
  /*public void moveSlider(double speed){
    if (frontSliderLimitSwitch.get() && rearSliderLimitSwitch.get()){//checks if either limit switch is pressed
      sliderMotor.set(speed);
    }
    else{
      if (!(frontSliderLimitSwitch.get()) && speed <= 0){//Ensures the slide doesn't extend when the limit switch is pressed
        sliderMotor.set(speed);
      }
      else if (!(rearSliderLimitSwitch.get()) && speed >= 0){//Ensures the slide doesn't retract when the limit switch is pressed
        sliderMotor.set(speed);
      }

      else{//Stops the motor 
        sliderMotor.set(0);
      }
      
    }
    
  }*/

  public static boolean isSliderBack(){
    return isBack;
    
  }


  
  //returns the current encoder ticks
  public double getSliderEncoderTicks(){
    return sliderEncoder.getPosition();
  }

  @Override
  public void periodic() {
    
    
    
  }
}
