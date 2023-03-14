// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;

public class MoveSliderCommand extends CommandBase {
  
  SliderSubsystem slider;
  XboxController control;
  boolean extended = false;

  public MoveSliderCommand(SliderSubsystem slider, XboxController auxControl) {
    this.slider = slider;
    this.control = auxControl;
    
    addRequirements(slider);
  }

  @Override
  public void execute() {
    
    if (control.getAButtonPressed()){
        extended = !extended;

     slider.set(extended ? SliderSubsystem.SliderPosition.EXTENDED : SliderSubsystem.SliderPosition.RETRACTED);
    }
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
