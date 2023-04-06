// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;

public class EndWhenObjectInClawCommand extends CommandBase {
  
  private double secondsToWait;
  private Timer timer = new Timer();

  public EndWhenObjectInClawCommand(double secondsToWait) {
    this.secondsToWait = secondsToWait;
  }

  @Override
  public void initialize(){
    timer.reset();
    timer.start();
  }
  
  @Override
  public boolean isFinished() {
    return timer.get() > secondsToWait && ClawIntakeSubsystem.isObjectInClaw();
  }
}
