// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;

public class RunIntakeCommand extends CommandBase {

  private final ClawIntakeSubsystem intake;
  private final double speed;
  private boolean intaken;
  
  
  public RunIntakeCommand(ClawIntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.set(speed);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
