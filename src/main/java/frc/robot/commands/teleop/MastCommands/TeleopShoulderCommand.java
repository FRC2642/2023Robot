// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;

public class TeleopShoulderCommand extends CommandBase {

  private final ShoulderSubsystem shoulder;
  private final XboxController auxControl;
  
  public TeleopShoulderCommand(ShoulderSubsystem shoulder, XboxController auxControl) {
    this.shoulder = shoulder;
    this.auxControl = auxControl;

    addRequirements(shoulder);
  }

  @Override
  public void initialize() {
    shoulder.setSpeedLimit(0.4);
    shoulder.setRampRate(1);
  }
  @Override
  public void execute() {
    double speed = auxControl.getLeftY();
    if(Math.abs(speed) < 0.1) shoulder.set(0);
    else shoulder.set(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}