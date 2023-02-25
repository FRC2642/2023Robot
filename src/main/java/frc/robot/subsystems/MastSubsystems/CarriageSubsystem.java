// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

public class CarriageSubsystem extends SubsystemBase {

  //creates motos and limit switches
  public CANSparkMax carriage = new CANSparkMax(Constants.CARRIAGE_MOTOR, MotorType.kBrushless);
  public static SparkMaxLimitSwitch carriageFrontLimitSwitch;
  public static SparkMaxLimitSwitch carriageBackLimitSwitch;
  public static RelativeEncoder carriageEncoder;


  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem() {
    carriageEncoder = carriage.getEncoder();
    carriageFrontLimitSwitch = carriage.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    carriageBackLimitSwitch = carriage.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  //motor direction not known must TEST!!!!1!11!1!!

  //moves the carriage unless its touching limit switches
  public void moveCarriage(double speed){
    if (speed > 0){
      if (carriageFrontLimitSwitch.isPressed()){
        carriage.set(0);
      }
      else {
        carriage.set(speed);
      }
    }
    else {
      if (carriageBackLimitSwitch.isPressed()){
        carriage.set(0);
      }
      else {
        carriage.set(speed);
      }
    }
  }

  //tells what position the carriage is out
  public static double getCarriageEncoder() {
    return carriageEncoder.getPosition();
  }

  //tests if carriage is at the end
  public static boolean isCarriageFullyExtended (){
    return carriageFrontLimitSwitch.isPressed();
  }
  
  //tests if carriage is at the begining
  public static boolean isCarriageFullyRetracted(){
    return carriageBackLimitSwitch.isPressed();
  }

  //reset the carriage position
  public void resetCarriageEncoder() {
    carriageEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
