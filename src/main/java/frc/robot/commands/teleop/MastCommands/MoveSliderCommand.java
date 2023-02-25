// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPositions;
import java.lang.Math;

public class MoveSliderCommand extends CommandBase {
  /** Creates a new MoveMainSliderCommand. */
  SliderSubsystem slider;
  XboxController auxController;

  public MoveSliderCommand(SliderSubsystem slider, XboxController auxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.slider = slider;
    this.auxController = auxController;
    
    addRequirements(slider);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //gets the button pressed on D-pad
    
    
    //stops slider form breaking by going up
    if (ShoulderSubsystem.getEncoderTicks() < 10 & ((Math.abs(ClawWristSubsystem.getEncoderTicks()) > 95) || (Math.abs(ClawWristSubsystem.getEncoderTicks()) < 85))){
    }else {
      SliderPositions dpadButton = slider.choosePosition(auxController.getPOV());
      slider.moveSlider(dpadButton);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double targetEncoderValue = slider.positions.get(slider.choosePosition(auxController.getPOV()));
    double currentEncoderValue = slider.getSliderEncoderTicks();
    
    return Math.abs(currentEncoderValue - targetEncoderValue) < .1;
  }
}