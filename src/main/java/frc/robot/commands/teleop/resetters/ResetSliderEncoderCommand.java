// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;

public class ResetSliderEncoderCommand extends CommandBase {

  private final SliderPosition pos;
  public ResetSliderEncoderCommand(SliderPosition position) {
    this.pos = position;
  }
  @Override
  public void execute() {
    SliderSubsystem.resetSliderEncoder(pos);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
