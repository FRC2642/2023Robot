// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SliderSubsystem extends SubsystemBase {
  /** Creates a new SliderSubsystem. */
  
  //Unsure whether or not the motor is inverted, make changes accordingly
  CANSparkMax sliderMotor = new CANSparkMax(Constants.MAIN_SLIDER_MOTOR, MotorType.kBrushless);
  RelativeEncoder sliderEncoder = sliderMotor.getEncoder();

  //It's likely that the switches return "True" when not pressed and vice versa, makes sure to test their Outputs
  DigitalInput frontSliderLimitSwitch = new DigitalInput(Constants.SLIDER_FRONT_LIMIT_SWITCH);
  DigitalInput rearSliderLimitSwitch = new DigitalInput(Constants.SLIDER_REAR_LIMIT_SWITCH);

  
  PIDController pid = new PIDController(0, 0, 0);

  
  

  public final HashMap <SliderPositions, Double> positions = new HashMap<SliderPositions, Double>();
  SliderPositions position;

  public SliderSubsystem() {
    positions.put(SliderPositions.FIRST_POSITION, 0.0);//Left on aux D-pad
    positions.put(SliderPositions.SECOND_POSITION, 10.0);//Up on aux D-pad
    //positions.put(SliderPositions.THIRD_POSITION, 10.0);//Right on aux D-pad
  }

  
  public void moveSlider(SliderPositions position){
    //Using the enum value it gets the targeted encoder value from the hashmap
    double targetEncoderPosition = positions.get(position); 
    
    //The speed of the slider motor
    double speed = MathUtil.clamp(pid.calculate(sliderEncoder.getPosition(), targetEncoderPosition), -1, 1);

    sliderMotor.set(speed);
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
    SECOND_POSITION,
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

  
  //returns the current encoder ticks
  public double getSliderEncoderTicks(){
    return sliderEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
