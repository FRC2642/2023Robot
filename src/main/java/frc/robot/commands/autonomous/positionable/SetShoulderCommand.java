// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;

public class SetShoulderCommand extends CommandBase {

  private final ShoulderSubsystem shoulder;
  private final Supplier<ShoulderSubsystem.ShoulderPosition> position;
  
  public SetShoulderCommand(ShoulderSubsystem shoulder, Supplier<ShoulderSubsystem.ShoulderPosition> position) {
    this.shoulder = shoulder;
    this.position = position;
    addRequirements(shoulder);
  }
  @Override
  public void initialize() {
    shoulder.setSpeedLimit(0.75);
    shoulder.setRampRate(1);
  }
  
  @Override
  public void end(boolean interrupted) {
    shoulder.set(0.0);
  }
  
  @Override
  public void execute() {
    shoulder.set(position.get());
  }

  @Override
  public boolean isFinished() {
    if (shoulder.atSetPosition()){
      System.out.println("Shoulder done");
    }
    return shoulder.atSetPosition();  
  }
}
