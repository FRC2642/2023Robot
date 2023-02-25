// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ClawWristSubsystem extends SubsystemBase {

  public CANSparkMax clawWristMotor;
  DigitalInput clawLimitSwitch = new DigitalInput(2);
  public static RelativeEncoder wristEncoder;
  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    clawWristMotor = new CANSparkMax(24, MotorType.kBrushed);
    wristEncoder = clawWristMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoder() {
    wristEncoder.setPosition(0);
  }

  public static double getEncoderTicks() {
    return wristEncoder.getPosition();
  }
  public void stopWrist() {
    clawWristMotor.set(0);
  }
  public void moveWrist(double speed) {
    clawWristMotor.set(speed);
  }
  public boolean getLimitSwitchState() {
    return clawLimitSwitch.get();
  }

  public boolean clawInRobot(){
    //checks if claw is inside sliders /*no number are correct*/
    return ((ShoulderSubsystem.getEncoderTicks() < 10 & ((CarriageSubsystem.getCarriageEncoder() > 90 ) || (!SliderSubsystem.isSliderBack()))) || 
    (ShoulderSubsystem.getEncoderTicks() > 170 & ((CarriageSubsystem.getCarriageEncoder() < 90) || SliderSubsystem.isSliderBack())));
  }

}
