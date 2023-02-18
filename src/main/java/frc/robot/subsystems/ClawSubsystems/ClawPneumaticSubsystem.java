// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawPneumaticSubsystem extends SubsystemBase {
  /** Creates a new ClawPneumaticSubsystem. */
  Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.GRIPPER_SOLENOID_CHANNEL);
  public ClawPneumaticSubsystem() {

  }

  public void gripperExtend() {
    gripperSolenoid.set(true);
  }
  
  public void gripperRetract() {
    gripperSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
