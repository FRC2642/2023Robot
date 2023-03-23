// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;

public class EnableShoulderBrokenCommand extends CommandBase {

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
  
  @Override
  public void execute() {
    ShoulderSubsystem.shoulderBroken = true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
