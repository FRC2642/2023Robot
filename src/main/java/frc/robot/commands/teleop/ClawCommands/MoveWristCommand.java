// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.utils.MathR;


public class MoveWristCommand extends CommandBase {
  private XboxController control;
  private ClawWristSubsystem wrist;
  private PIDController pid = new PIDController(0.2, 0, 0);
  private String direction = "center";
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

    double speed = 0;
    if (control.getPOV() == 0){
      direction = "center";
      speed = MathR.limit(pid.calculate(ClawWristSubsystem.getEncoderTicks(), 0), -0.8, 0.8);
    }
    else if (control.getPOV() == 90){
      direction = "right";
      speed = MathR.limit(pid.calculate(ClawWristSubsystem.getEncoderTicks(), 10), -0.8, 0.8);
    }
    else if (control.getPOV() == 270){
      direction = "left";
      speed = MathR.limit(pid.calculate(ClawWristSubsystem.getEncoderTicks(), -10), -0.8, 0.8);
    }
    else if (control.getPOV() == 45){
      direction = "topRight";
      speed = MathR.limit(pid.calculate(ClawWristSubsystem.getEncoderTicks(), 5), -0.8, 0.8);
    }
    else if (control.getPOV() == 315){
      direction = "topLeft";
      speed = MathR.limit(pid.calculate(ClawWristSubsystem.getEncoderTicks(), -5), -0.8, 0.8);
    }


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
