// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
  /** Creates a new ShoulderSubsystem. */
  CANSparkMax shoulderMotor = new CANSparkMax(23, MotorType.kBrushed);

  DigitalInput frontShoulderLimitSwitch = new DigitalInput(5);
  DigitalInput backShoulderLimitSwitch = new DigitalInput(6);
  public ShoulderSubsystem() {}

  public void moveShoulder(double speed){
    
    if(frontShoulderLimitSwitch.get() && backShoulderLimitSwitch.get()){//checks which switch is being pressed
      shoulderMotor.set(speed);
    }
    else{
      if(!(frontShoulderLimitSwitch.get()) && speed <= 0){//Ensures that the motor stops moving towards the direction of the pressed switch
        shoulderMotor.set(speed);
      }
      else if(!(backShoulderLimitSwitch.get()) && speed >=0){//Ensures that the motor stops moving towards the direction of the pressed switch
        shoulderMotor.set(speed);
      }
      else{//Makes the motor stop
        shoulderMotor.set(0);
      }

      }
    }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
