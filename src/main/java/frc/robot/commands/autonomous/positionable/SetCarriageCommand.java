// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;

public class SetCarriageCommand extends CommandBase {

  private final CarriageSubsystem carriage;
  private final Supplier<CarriageSubsystem.CarriagePosition> position;

  public SetCarriageCommand(CarriageSubsystem carriage, Supplier<CarriageSubsystem.CarriagePosition> position) {
    this.carriage = carriage;
    this.position = position;
    addRequirements(carriage);
  }

  @Override
  public void initialize() {
    carriage.setSpeedLimit(0.6);
    carriage.setRampRate(1.0);
  }

  @Override
  public void execute() {
    carriage.set(position.get());
  }

  @Override
  public boolean isFinished() {
    return carriage.atSetPosition();
  }
}
