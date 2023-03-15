// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;

public class ResetWristEncoderCommand extends CommandBase {

  private final ClawWristSubsystem wrist;
  private final ClawWristSubsystem.WristPosition position;

  public ResetWristEncoderCommand(ClawWristSubsystem wrist, WristPosition position) {
      this.wrist = wrist;
      this.position = position;
    addRequirements(wrist);
  }

  @Override
  public void execute() {
    wrist.resetWristEncoder(position);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
