// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;

public class SetCarriageCommand extends CommandBase {

  private final CarriageSubsystem carriage;
  private final CarriageSubsystem.CarriagePosition position;

  public SetCarriageCommand(CarriageSubsystem carriage, CarriageSubsystem.CarriagePosition position) {
    this.carriage = carriage;
    this.position = position;
    addRequirements(carriage);
  }

  @Override
  public void execute() {
    carriage.set(position);
  }

  @Override
  public boolean isFinished() {
    return carriage.atSetPosition();
  }
}
