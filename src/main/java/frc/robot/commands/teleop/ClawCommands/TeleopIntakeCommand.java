// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;

public class TeleopIntakeCommand extends CommandBase {

  private final ClawIntakeSubsystem intake;
  private final XboxController auxControl;

  public TeleopIntakeCommand(ClawIntakeSubsystem intake, XboxController auxControl) {
    this.intake = intake;
    this.auxControl = auxControl;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    double speed = 0;

    if (auxControl.getRightTriggerAxis() > 0.2) {
      speed = Math.pow(auxControl.getRightTriggerAxis(), 2);
    } 
    else if (auxControl.getLeftTriggerAxis() > 0.2) {
      speed = -Math.pow(auxControl.getLeftTriggerAxis(), 2);
    }
    if (ClawGripperSubsystem.isOpen() && speed < 0) {
      speed *= 0.54;
    }
    if (ClawGripperSubsystem.isOpen() && ClawIntakeSubsystem.isObjectInClaw() && auxControl.getLeftTriggerAxis() <= 0.1) {
      speed = 0.102;
    }
    
    intake.set(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
