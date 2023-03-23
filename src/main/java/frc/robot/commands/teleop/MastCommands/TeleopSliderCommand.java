// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;

public class TeleopSliderCommand extends CommandBase {
  
  private final SliderSubsystem slider;
  private final XboxController auxControl;

  private boolean extended = false;

  public TeleopSliderCommand(SliderSubsystem slider, XboxController auxControl) {
    this.slider = slider;
    this.auxControl = auxControl;
    addRequirements(slider);
  }

  @Override
  public void initialize() {
    slider.setRampRate(0.5);
    slider.setSpeedLimit(1);
  }

  @Override
  public void execute() {
    if (!ShoulderSubsystem.shoulderBroken) slider.set(0.0);
    else if (Math.abs(auxControl.getLeftY()) > 0.1) slider.setManual(auxControl.getLeftY() * -1);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
