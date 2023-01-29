// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  
  

    CANSparkMax FLMotor = new CANSparkMax(Constants.FL, MotorType.kBrushless);
    CANSparkMax BLMotor = new CANSparkMax(Constants.BL, MotorType.kBrushless);
    CANSparkMax FRMotor = new CANSparkMax(Constants.FR, MotorType.kBrushless);
    CANSparkMax BRMotor = new CANSparkMax(Constants.BR, MotorType.kBrushless);

    RelativeEncoder FRencoder = FRMotor.getEncoder();

    MotorControllerGroup RightMotors= new MotorControllerGroup(FRMotor, BRMotor);
    MotorControllerGroup LeftMotors= new MotorControllerGroup(FLMotor, BLMotor);

    DifferentialDrive DDrive= new DifferentialDrive(LeftMotors, RightMotors);
    
  public DriveSubsystem(){
    LeftMotors.setInverted(true);
  }
  
  public void arcadeDrive(double movespeed, double rotatespeed) {
    DDrive.arcadeDrive(movespeed, rotatespeed);
  }

  public double getEncoderTicks(){

    return FRencoder.getPosition();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}