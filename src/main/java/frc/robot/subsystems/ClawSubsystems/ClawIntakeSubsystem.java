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
  //Wheel motors and limit switch ID
  CANSparkMax gripperMotor = new CANSparkMax(Constants.GRIPPER_INTAKE_MOTOR, MotorType.kBrushless);
  DigitalInput gripperLimitSwitch = new DigitalInput(0);
  public boolean intakeMode = true;
  
  public ClawIntakeSubsystem() {}
  //Sets the speed at which the wheels spin
  public void runGripperIntake(double speed) {
    gripperMotor.set(speed);
  }

  public void outtakeGripperIntake(double speed){
    gripperMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
