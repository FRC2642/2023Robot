// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;

public class SetShoulderCommand extends CommandBase {

  private final ShoulderSubsystem shoulder;
  private final ShoulderSubsystem.ShoulderPosition position;
  
  public SetShoulderCommand(ShoulderSubsystem shoulder, ShoulderSubsystem.ShoulderPosition position) {
    this.shoulder = shoulder;
    this.position = position;
    addRequirements(shoulder);
  }
  @Override
  public void initialize() {
    shoulder.setSpeedLimit(0.35);
    shoulder.setRampRate(0.0);
  }
  
  @Override
  public void execute() {
    shoulder.set(position);
  }

  @Override
  public boolean isFinished() {
    return shoulder.atSetPosition();
  }
}
