// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

public class CarriageSubsystem extends SubsystemBase {

  //creates motos and limit switches
  public CANSparkMax carriage = new CANSparkMax(Constants.CARRIAGE_MOTOR, MotorType.kBrushless);
  
  public static RelativeEncoder carriageEncoder;
  public Solenoid brake = ClawPneumaticSubsystem.pneumatics.makeSolenoid(1);

  Timer lagTimer = new Timer();


  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem() {
    carriageEncoder = carriage.getEncoder();
  }


  //moves the carriage unless its touching limit switches
  public void move(double speed){
    /*if (Math.abs(speed) < 0.2){
      carriage.set(0);
    }
    else{
      carriage.set(-speed);
    }*/
    carriage.set(speed);
    
    
  }

  

  //tells what position the carriage is out
  public static double getCarriageEncoder() {
    return carriageEncoder.getPosition();
  }

  //reset the carriage position
  public void resetCarriageEncoder() {
    carriageEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("carriage encoder: ",carriageEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
