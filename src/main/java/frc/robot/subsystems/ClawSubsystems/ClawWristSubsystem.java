// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ClawWristSubsystem extends SubsystemBase {

  public CANSparkMax wrist = new CANSparkMax(24, MotorType.kBrushed);
  public static RelativeEncoder wristEncoder;
  public static SparkMaxLimitSwitch wristLimitSwitch;

  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    //wristEncoder = wrist.getEncoder();
    wristLimitSwitch = wrist.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
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
  public void stop() {
    wrist.set(0);
  }
  public void move(double speed) {
    // If limit switch is hit, prevents wrist from moving wrist in the same direction further
    //if (wrist.getLimitSwitchState() == false) {
      // prevents wrist from twisting inside of robot
      if (!clawInRobot()){
        if (ClawWristSubsystem.getEncoderTicks() > -180 && speed <= -0.1) {
          wrist.set(speed);
        }
        else if (ClawWristSubsystem.getEncoderTicks() < 180 && speed >= 0.1) {
          wrist.set(speed);
        }
        else {
          stop();
        }
      }
      else{
        stop();
      }
    //} 
  }
  public boolean getLimitSwitchState() {
    return wristLimitSwitch.isPressed();
  }

  public boolean clawInRobot(){
    //checks if claw is inside sliders /*no number are correct*/
    return ((ShoulderSubsystem.getEncoderTicks() < 10 && ((CarriageSubsystem.getCarriageEncoder() > 90 ) || (!SliderSubsystem.isSliderBack()))) || 
    (ShoulderSubsystem.getEncoderTicks() > 170 & ((CarriageSubsystem.getCarriageEncoder() < 90) || SliderSubsystem.isSliderBack())));
  }

}
