// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;

public class SetSliderCommand extends CommandBase {
  
  private final SliderSubsystem sliders;
  private final SliderSubsystem.SliderPosition position;

  public SetSliderCommand(SliderSubsystem sliders, SliderSubsystem.SliderPosition position) {
    this.sliders = sliders;
    this.position = position;
    addRequirements(sliders);
  }
  @Override
  public void initialize() {
    sliders.setSpeedLimit(1.0);
    sliders.setRampRate(0.0);
  }

  @Override
  public void execute() {
    sliders.set(position);
  }

  @Override
  public boolean isFinished() {
    return sliders.atSetPosition();
  }
}
