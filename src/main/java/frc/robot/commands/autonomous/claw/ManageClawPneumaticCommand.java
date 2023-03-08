// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawPneumaticSubsystem;

public class ManageClawPneumaticCommand extends CommandBase {
  /** Creates a new ManageClawPneumaticCommand. */
  ClawPneumaticSubsystem pneumatics;
  boolean open;
  public ManageClawPneumaticCommand(ClawPneumaticSubsystem pneumatics, boolean open) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pneumatics = pneumatics;
    this.open = open;
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!open){
      pneumatics.gripperRetract();
    }
    else{
      pneumatics.gripperExtend();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
