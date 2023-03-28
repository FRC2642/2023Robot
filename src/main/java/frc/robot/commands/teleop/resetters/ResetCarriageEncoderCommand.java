// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;

public class ResetCarriageEncoderCommand extends CommandBase {

  private final CarriagePosition pos;
  public ResetCarriageEncoderCommand(CarriagePosition position) {
    this.pos = position;
  }

  @Override
  public void execute() {
    CarriageSubsystem.resetCarriageEncoder(pos);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
