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

public class ShoulderSubsystem extends SubsystemBase {
  /** Creates a new ShoulderSubsystem. */
  CANSparkMax shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushless);

  DigitalInput frontShoulderLimitSwitch = new DigitalInput(Constants.SHOULDER_FRONT_LIMIT_SWITCH);
  DigitalInput rearShoulderLimitSwitch = new DigitalInput(Constants.SHOULDER_REAR_LIMIT_SWITCH);

  static RelativeEncoder shoulderEncoder;

  public ShoulderSubsystem() {
    shoulderEncoder = shoulderMotor.getEncoder();
  }

  public void moveShoulder(double speed){
    
    if(frontShoulderLimitSwitch.get() && rearShoulderLimitSwitch.get()){//checks which switch is being pressed
      shoulderMotor.set(speed);
    }
    else{
      if(!(frontShoulderLimitSwitch.get()) && speed <= 0){//Ensures that the motor stops moving towards the direction of the pressed switch
        shoulderMotor.set(speed);
      }
      else if(!(rearShoulderLimitSwitch.get()) && speed >=0){//Ensures that the motor stops moving towards the direction of the pressed switch
        shoulderMotor.set(speed);
      }
      else{//Makes the motor stop
        shoulderMotor.set(0);
      }

      }
    }

    public static double getEncoderTicks(){
      return shoulderEncoder.getPosition();
    }

    public void resetEncoder(){
      shoulderEncoder.setPosition(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
