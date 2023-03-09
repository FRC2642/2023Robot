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
  /** Creates a new MoveMainSliderCommand. */
  SliderSubsystem slider;
  XboxController control;
  PIDController pid = new PIDController(0.2, 0, 0);
  boolean extended = false;

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
    //moves slider if A button is pressed
    if (control.getAButtonPressed()){
      //dont move illegaly moving down
      if (ShoulderSubsystem.getEncoderTicks() < 90 && extended){
        extended = !extended;
      //dont move illegaly moving up
     } else if (ShoulderSubsystem.getEncoderTicks() > 90 && !extended){
        extended = !extended;
     }

     //move slider
     slider.move(extended);
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
