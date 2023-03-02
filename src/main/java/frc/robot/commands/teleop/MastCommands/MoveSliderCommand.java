// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;

public class MoveSliderCommand extends CommandBase {
  /** Creates a new MoveMainSliderCommand. */
  SliderSubsystem slider;
  XboxController control;
  PIDController pid = new PIDController(0.2, 0, 0);
  boolean extending = false;

  public MoveSliderCommand(SliderSubsystem slider, XboxController auxControl) {
    this.slider = slider;
    this.control = auxControl;
    
    addRequirements(slider);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if (control.getAButtonPressed()){
        extending = !extending;
     }

     slider.move(extending);
    
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
