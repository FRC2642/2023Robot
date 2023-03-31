// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawIntakeSubsystem extends SubsystemBase {
  
  private final CANSparkMax intake = new CANSparkMax(Constants.GRIPPER_INTAKE_MOTOR, MotorType.kBrushless);
  private static SparkMaxLimitSwitch intakeLimitSwitch;
  
  public ClawIntakeSubsystem() {
    intake.setInverted(false);
    intakeLimitSwitch = intake.getForwardLimitSwitch(Type.kNormallyOpen);
  }

  //Positive = intake, Negative = outake
  public void set(double speed) {
    intake.set(speed);
 //   System.out.println("CLAW SPEED: " + speed);
  }

  public static boolean isObjectInClaw(){
    return intakeLimitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Object In Claw", isObjectInClaw());
   //if (getCurrentCommand() != null) SmartDashboard.putString("COMMAND", getCurrentCommand().getName() );
    // This method will be called once per scheduler run
  }
}
