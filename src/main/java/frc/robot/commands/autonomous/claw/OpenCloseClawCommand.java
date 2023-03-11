// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;

public class OpenCloseClawCommand extends CommandBase {
  
  private final ClawGripperSubsystem pneumatics;
  private final boolean open;

  public OpenCloseClawCommand(ClawGripperSubsystem pneumatics, boolean open) {
    this.pneumatics = pneumatics;
    this.open = open;
    addRequirements(pneumatics);
  }

  @Override
  public void execute() {
    pneumatics.set(open);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
