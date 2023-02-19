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

  //It's likely that the switches return "True" when not pressed and vice versa, makes sure to test their Outputs
  DigitalInput frontSliderLimitSwitch = new DigitalInput(Constants.SLIDER_FRONT_LIMIT_SWITCH);
  DigitalInput rearSliderLimitSwitch = new DigitalInput(Constants.SLIDER_REAR_LIMIT_SWITCH);

  PIDController pid = new PIDController(0, 0, 0);

  
  RelativeEncoder sliderEncoder = sliderMotor.getEncoder();

  public final HashMap <Integer, Double> positions = new HashMap<Integer, Double>();

  public SliderSubsystem() {
    positions.put(270, 0.0);//Left on D-pad
    positions.put(0, 5.0);//Up on D-pad
    positions.put(90, 10.0);//Right on D-pad
  }

  
  public void moveSlider(int DpadInput){
    //The targeted encoder tick value
    double targetEncoderPosition = positions.get(DpadInput); 
    
    //The speed of the sldier motor
    double speed = MathUtil.clamp(pid.calculate(sliderEncoder.getPosition(), targetEncoderPosition), -1, 1);

    sliderMotor.set(speed);
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

  

  public double getSliderEncoderTicks(){
    return sliderEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
