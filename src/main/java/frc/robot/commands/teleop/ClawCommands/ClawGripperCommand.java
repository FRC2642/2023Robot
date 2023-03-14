// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;

public class ClawGripperCommand extends CommandBase {
  
  private final ClawGripperSubsystem pneumatics;
  private final XboxController auxControl;

  public ClawGripperCommand(ClawGripperSubsystem pneumatics, XboxController auxControl) { 
    this.pneumatics = pneumatics;
    this.auxControl = auxControl;
    addRequirements(pneumatics);
  }

  @Override
  public void execute() {
      if (auxControl.getLeftBumper()) {
        pneumatics.set(true);
      }
  
      else if (auxControl.getRightBumper()) {
        pneumatics.set(false);
      }
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
