// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import java.lang.Math;

public class MoveShoulder extends CommandBase {
  ShoulderSubsystem shoulder;
  XboxController auxController;
  /** Creates a new MoveShoulder. */
  public MoveShoulder(ShoulderSubsystem shoulder, XboxController auxController) {
    this.shoulder = shoulder;
    this.auxController = auxController;

    addRequirements(shoulder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //is shoulder up
    if (shoulder.getEncoderTicks() > 170){

      //if wrist is in safe rotation area
      if ((Math.abs(ClawWristSubsystem.getEncoderTicks()) > 85 & Math.abs(ClawWristSubsystem.getEncoderTicks()) < 95)){
        double speed = auxController.getRightY();
        shoulder.moveShoulder(speed);

      //if wrist is not in safe roatation but your going to safe way
      } else if (auxController.getRightY() < 0){
        double speed = auxController.getRightY();
        shoulder.moveShoulder(speed);
      }

      //is shoulder down
    } else if (shoulder.getEncoderTicks() < 10) {

      //is wrist in safe rotation area
      if ((Math.abs(ClawWristSubsystem.getEncoderTicks()) > 85 & Math.abs(ClawWristSubsystem.getEncoderTicks()) < 95)){
        double speed = auxController.getRightY();
        shoulder.moveShoulder(speed);

        //if wrist isnt good but your going the safe way
      } else if (auxController.getRightY() > 0){
        double speed = auxController.getRightY();
        shoulder.moveShoulder(speed);
      }
    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}