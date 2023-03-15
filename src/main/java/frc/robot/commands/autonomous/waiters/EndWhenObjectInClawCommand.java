// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.waiters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;

public class EndWhenObjectInClawCommand extends CommandBase {
  private final boolean endInClaw;

  public EndWhenObjectInClawCommand(boolean endInClaw) {
    this.endInClaw = endInClaw;
  }
  
  @Override
  public boolean isFinished() {
    return endInClaw && ClawIntakeSubsystem.isObjectInClaw();
  }
}
