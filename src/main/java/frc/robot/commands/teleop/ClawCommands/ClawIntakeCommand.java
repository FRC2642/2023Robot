// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;


public class ClawIntakeCommand extends CommandBase {
  ClawIntakeSubsystem intake;
  XboxController mainControl;
  XboxController auxControl;
  /** Creates a new RunIntakeCommand. */
  public ClawIntakeCommand(ClawIntakeSubsystem intake, XboxController mainControl, XboxController auxControl) {
    this.intake = intake;
    this.mainControl = mainControl;
    this.auxControl = auxControl;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /*   if (SliderSubsystem.isSliderBack() && CarriageSubsystem.isCarriageFullyRetracted()){
      if (mainControl.getStartButton()){
        intake.intakeMode = !intake.intakeMode;
      }

      if (intake.intakeMode){
        intake.runGripperIntake(mainControl.getRightTriggerAxis() * .8);
      }
      else{
        intake.outtakeGripperIntake(mainControl.getRightTriggerAxis() * .8);
      }
      
    }
    else{
      if (auxControl.getLeftTriggerAxis() < 0.1){
        intake.runGripperIntake(mainControl.getRightTriggerAxis() * .8);
      }
      else if (auxControl.getRightTriggerAxis() < 0.1){
        intake.outtakeGripperIntake(auxControl.getLeftTriggerAxis()*.8);
      }
    }*/
    if (auxControl.getRightTriggerAxis() >= 0.1){
      intake.runGripperIntake(auxControl.getRightTriggerAxis());
    }
    else if (auxControl.getLeftTriggerAxis() >= 0.1){
      intake.runGripperIntake(-auxControl.getLeftTriggerAxis());
    }
    else{
      intake.runGripperIntake(0);
    }
    
  }

  // Called once the command ends or is interrupted.2
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
