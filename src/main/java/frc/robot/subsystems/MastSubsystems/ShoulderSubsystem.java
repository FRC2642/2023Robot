// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkMaxLimitSwitch;

public class ShoulderSubsystem extends SubsystemBase {
  /** Creates a new ShoulderSubsystem. */
  CANSparkMax shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushed);

  private static SparkMaxLimitSwitch frontShoulderLimitSwitch;
  private static SparkMaxLimitSwitch rearShoulderLimitSwitch;

  static RelativeEncoder shoulderEncoder;

  public ShoulderSubsystem() {
    //shoulderEncoder = shoulderMotor.getEncoder();
    frontShoulderLimitSwitch = shoulder.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rearShoulderLimitSwitch = shoulder.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  public boolean getFrontShoulderLimitSwitch(){
    return frontShoulderLimitSwitch.isPressed();
  }

  public boolean getRearShoulderLimitSwitch(){
    return rearShoulderLimitSwitch.isPressed();
  }

  public void move(double speed){
    if(!getFrontShoulderLimitSwitch() && !getRearShoulderLimitSwitch() && Math.abs(speed) >= 0.1){
      //Move if limit switches are false
      shoulder.set(speed);
    }
    else if(getFrontShoulderLimitSwitch() && speed <= -0.1){
        //Dont move further than front switch
        shoulder.set(speed);
      }
    else if(getRearShoulderLimitSwitch() && speed >= 0.1){
      //Dont move further than rear switch
      shoulder.set(speed);
    }
    else{
      //Stop
      shoulder.set(speed);
    }
      
  }

    public static boolean isShoulderBack(){
      return rearShoulderLimitSwitch.isPressed();
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
