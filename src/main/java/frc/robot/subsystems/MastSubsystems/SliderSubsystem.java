// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SliderSubsystem extends SubsystemBase {
  /** Creates a new SliderSubsystem. */

  CANSparkMax sliderMotor = new CANSparkMax(Constants.MAIN_SLIDER_MOTOR, MotorType.kBrushless);
  DigitalInput frontSliderLimitSwitch = new DigitalInput(Constants.SLIDER_FRONT_LIMIT_SWITCH);
  private static DigitalInput rearSliderLimitSwitch = new DigitalInput(Constants.SLIDER_REAR_LIMIT_SWITCH);
  

  //It's likely that the switch returns "True" when not pressed and vice versa
  RelativeEncoder sliderEncoder = sliderMotor.getEncoder();

  public SliderSubsystem() {}

  public void moveSlider(double speed){
    
    if (frontSliderLimitSwitch.get() && rearSliderLimitSwitch.get()){
      sliderMotor.set(speed);
    }
    else{
      if (!(frontSliderLimitSwitch.get()) && speed <= 0){//Ensures the slide doesn't extend when the limit switch is pressed
        sliderMotor.set(speed);
      }
      else if (!(rearSliderLimitSwitch.get()) && speed >= 0){//Ensures the slide doesn't retract when the limit switch is pressed
        sliderMotor.set(speed);
      }

      else{
        sliderMotor.set(0);
      }
      
    }
    
  }

  public static boolean isSliderBack(){
    return !rearSliderLimitSwitch.get();
  }


  public double getSliderEncoderTicks(){
    return sliderEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
