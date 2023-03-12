// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawIntakeSubsystem extends SubsystemBase {
  CANSparkMax intake = new CANSparkMax(Constants.GRIPPER_INTAKE_MOTOR, MotorType.kBrushless);
  DigitalInput intakeLimitSwitch = new DigitalInput(0);
  
  public ClawIntakeSubsystem() {
    intake.setInverted(false);
  }

  //Positive = intake, Negative = outake
  public void set(double speed) {
    intake.set(speed);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
