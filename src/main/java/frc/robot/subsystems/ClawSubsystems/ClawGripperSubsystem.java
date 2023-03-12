// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawGripperSubsystem extends SubsystemBase {
  
  public static final PneumaticHub pneumatics = new PneumaticHub(32);
  private static Solenoid gripperSolenoid;

  public ClawGripperSubsystem() {
    pneumatics.enableCompressorAnalog(100, 120);
    gripperSolenoid = pneumatics.makeSolenoid(Constants.GRIPPER_SOLENOID_CHANNEL);
  }

  public void set(boolean open) {
    gripperSolenoid.set(open);
  }

  public static boolean isOpen(){
    return gripperSolenoid.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
