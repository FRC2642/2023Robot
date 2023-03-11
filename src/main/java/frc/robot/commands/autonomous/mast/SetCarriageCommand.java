// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.mast;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;

public class SetCarriageCommand extends CommandBase {

  private final CarriageSubsystem shoulder;
  private final CarriageSubsystem.CarriagePosition position;

  public SetCarriageCommand(CarriageSubsystem shoulder, CarriageSubsystem.CarriagePosition position) {
    this.shoulder = shoulder;
    this.position = position;
    addRequirements(shoulder);
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
