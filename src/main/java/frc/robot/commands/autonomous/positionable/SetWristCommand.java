// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;

public class SetWristCommand extends CommandBase {

  private final ClawWristSubsystem wrist;
  private final Supplier<ClawWristSubsystem.WristPosition> position;

  public SetWristCommand(ClawWristSubsystem wrist, Supplier<ClawWristSubsystem.WristPosition> position, double threshold) {
    this.wrist = wrist;
    this.position = position;
    ClawWristSubsystem.AT_SETPOINT_THRESHOLD = threshold;
    addRequirements(wrist);
  }
  public SetWristCommand(ClawWristSubsystem wrist, Supplier<ClawWristSubsystem.WristPosition> position) {
    this.wrist = wrist;
    this.position = position;
    ClawWristSubsystem.AT_SETPOINT_THRESHOLD = 5d;
    addRequirements(wrist);
  }
  @Override
  public void initialize() {
    wrist.setSpeedLimit(1.0);
    wrist.setRampRate(0);
  }

  @Override
  public void execute() {
    wrist.set(position.get());
  }
  
  @Override
  public void end(boolean interrupted) {
    wrist.set(0.0);
  }

  @Override
  public boolean isFinished() {
    
    return wrist.atSetPosition();
  }
}
