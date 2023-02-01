// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax frontLeft = new CANSparkMax(Constants.frontLeftID, MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(Constants.frontRightID, MotorType.kBrushless);
  CANSparkMax backLeft = new CANSparkMax(Constants.backLeftID, MotorType.kBrushless);
  CANSparkMax backRight = new CANSparkMax(Constants.backRightID, MotorType.kBrushless);

  MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeft, backLeft);
  MotorControllerGroup rightGroup = new MotorControllerGroup(frontRight, backRight);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
  
  /** Creates a new DrivingExample. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void drive(double speed, double turn) {
    differentialDrive.arcadeDrive(speed, turn);
  }
}

