// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawIntakeSubsystem extends SubsystemBase {
  CANSparkMax intake = new CANSparkMax(Constants.GRIPPER_INTAKE_MOTOR, MotorType.kBrushless);
  
  
  public ClawIntakeSubsystem() {

  }

  public void move(double speed) {
    if (speed <= -0.1){
      intake.set(speed);
      
    }
    else if (speed >= 0.1){
      intake.set(speed);
    }
    else{
      intake.set(0);
    }
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(intake.getForwardLimitSwitch(Type.kNormallyOpen).isPressed());
  }
}
