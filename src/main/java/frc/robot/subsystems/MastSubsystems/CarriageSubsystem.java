// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CarriageSubsystem extends SubsystemBase {

  //creates motos and limit switches
  public CANSparkMax carriage = new CANSparkMax(Constants.CARRIAGE_MOTOR, MotorType.kBrushless);
  private static DigitalInput carriageFrontLimitSwitch = new DigitalInput(Constants.CARRIAGE_FRONT_LIMIT_SWITCH);
  private static DigitalInput carriageBackLimitSwitch = new DigitalInput(Constants.CARRIAGE_BACK_LIMIT_SWITCH);
  public static RelativeEncoder carriageEncoder;


  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem() {
    carriageEncoder = carriage.getEncoder();
  }

  //motor direction not known must TEST!!!!1!11!1!!

  //moves the carriage unless its touching limit switches
  public void moveCarriage(double speed){
    if (speed > 0){
      if (carriageFrontLimitSwitch.get()){
        carriage.set(0);
      }
      else {
        carriage.set(speed);
      }
    }
    else {
      if (carriageBackLimitSwitch.get()){
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
    return !carriageFrontLimitSwitch.get();
  }
  
  //tests if carriage is at the begining
  public static boolean isCarriageFullyRetracted(){
    return !carriageBackLimitSwitch.get();
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
