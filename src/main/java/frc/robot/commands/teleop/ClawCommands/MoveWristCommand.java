// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;


public class MoveWristCommand extends CommandBase {
  private XboxController control;
  private ClawWristSubsystem wrist;
  /** Creates a new ClawWristDirection. */
  public MoveWristCommand(ClawWristSubsystem wrist, XboxController auxControl) {
    this.control = auxControl;
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.resetEncoder();
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (limelight.getDetectionType() == "CONE"){
      if (limelight.getWidth() - limelight.getHeight() <= 1){
        wrist.moveWrist(-1);
      }
    }*/

    double speed = control.getLeftX();

    wrist.move(speed);
    
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
