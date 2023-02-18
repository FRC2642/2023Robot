// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
  /** Creates a new ShoulderSubsystem. */
  CANSparkMax shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushed);

  DigitalInput frontShoulderLimitSwitch = new DigitalInput(Constants.SHOULDER_FRONT_LIMIT_SWITCH);
  DigitalInput rearShoulderLimitSwitch = new DigitalInput(Constants.SHOULDER_REAR_LIMIT_SWITCH);

  public ShoulderSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
