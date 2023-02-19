// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawPneumaticSubsystem;


public class ClawPneumaticCommand extends CommandBase {
  /** Creates a new ClawPneumaticCommand. */ 
  ClawPneumaticSubsystem pneumatics;
  XboxController mainControl;
  XboxController auxControl;
  public ClawPneumaticCommand(ClawPneumaticSubsystem pneumatics, XboxController mainControl, XboxController auxControl) { 
    this.pneumatics = pneumatics;
    this.mainControl = mainControl;
    this.auxControl = auxControl;
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mainControl.getYButton()) {
      pneumatics.gripperExtend();
    }

    else if (mainControl.getBButton()) {
      pneumatics.gripperRetract();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
