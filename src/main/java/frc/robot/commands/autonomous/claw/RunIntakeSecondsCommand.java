// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;

public class RunIntakeSecondsCommand extends CommandBase {
  /** Creates a new RunIntakeSecondsCommand. */
  ClawIntakeSubsystem intake;
  double seconds;
  boolean cone;
  Timer timer = new Timer();
  public RunIntakeSecondsCommand(ClawIntakeSubsystem intake, double seconds, boolean cone) {
    this.intake = intake;
    this.seconds = seconds;
    this.cone = cone;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset(); 
    timer.start();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cone){
      intake.move(1);
    }
    else{
      intake.move(0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > seconds;
  }
}
