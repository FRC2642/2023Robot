// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;

import com.revrobotics.SparkMaxLimitSwitch;


public class SliderSubsystem extends SubsystemBase {
  /** Creates a new SliderSubsystem. */
  
  //Unsure whether or not the motor is inverted, make changes accordingly
  CANSparkMax slider = new CANSparkMax(Constants.MAIN_SLIDER_MOTOR, MotorType.kBrushless);
  RelativeEncoder sliderEncoder = slider.getEncoder();


  public static SparkMaxLimitSwitch frontSliderLimitSwitch;
  public static SparkMaxLimitSwitch rearSliderLimitSwitch;

  public static final HashMap <SliderPositions, Double> positions = new HashMap<SliderPositions, Double>();
  SliderPositions position;

  public SliderSubsystem() {
    positions.put(SliderPositions.FIRST_POSITION, 0.0);//Left on aux D-pad
    positions.put(SliderPositions.SECOND_POSITION, 10.0);//Up on aux D-pad
    //positions.put(SliderPositions.THIRD_POSITION, 10.0);//Right on aux D-pad

    frontSliderLimitSwitch = slider.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rearSliderLimitSwitch = slider.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  
  public void move(double speed){
    //stops slider from breaking by going up
    /*if (ShoulderSubsystem.getEncoderTicks() < 10 & ((Math.abs(ClawWristSubsystem.getEncoderTicks()) > 95) || (Math.abs(ClawWristSubsystem.getEncoderTicks()) < 85))){
      slider.set(0);
    }
    else {
      slider.set(speed);
    }*/
    slider.set(speed);
  }
  //returns the position to go to based on D-pad input
  public SliderPositions choosePosition(int dPadInput){
    switch (dPadInput){
      case 270://Left on aux D-Pad
      position = SliderPositions.FIRST_POSITION;
      break;

      case 0://Up on aux D Pad
      position = SliderPositions.SECOND_POSITION;
      break;

      /*case 90://Right on aux D-Pad
      position = SliderPositions.THIRD_POSITION;
      break;*/

    }

    return position;
  }
    
  public enum SliderPositions {
    FIRST_POSITION,
    SECOND_POSITION
    //THIRD_POSITION

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
    return rearSliderLimitSwitch.isPressed();
  }


  
  //returns the current encoder ticks
  public double getSliderEncoderTicks(){
    return sliderEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
