// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.utils.MathR;


public class ClawWristCommand extends CommandBase {
  private XboxController control;
  private ClawWristSubsystem wrist;
  private PIDController pid = new PIDController(0.01, 0, 0.0);
  private String direction = "center";
  /** Creates a new ClawWristDirection. */
  public ClawWristCommand(ClawWristSubsystem wrist, XboxController auxControl) {
    this.control = auxControl;
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.resetWristEncoder();
    pid.setSetpoint(180);
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (limelight.getDetectionType() == "CONE"){
      if (limelight.getWidth() - limelight.getHeight() <= 1){
        wrist.moveWrist(-1);
      }
    }*/
    
    if (control.getPOV() == 0){
      direction = "center";
      pid.setSetpoint(180);
    }
    else if (control.getPOV() == 90
    ){
      direction = "right";
      pid.setSetpoint(92);
    }
    else if (control.getPOV() == 270){
      direction = "left";
      pid.setSetpoint(270);
    }
    else if (control.getPOV() == 45){
      direction = "topRight";
      pid.setSetpoint(125);
    }
    else if (control.getPOV() == 315){
      direction = "topLeft";
      pid.setSetpoint(225);
    }
    else if (control.getPOV() == 180){
      direction = "bottom";
      pid.setSetpoint(360);
    }
    
   /*  if (control.getXButton()){
      speed = 0.4;
    }
    else if (control.getYButton()){
      speed = -0.4;
    }*/
    
    //System.out.println("direction " + direction);

    //wrist.move(speed);
    double speed =  MathR.limit(-pid.calculate(wrist.getEncoderTicks()), -0.5, 0.5);
    //SmartDashboard.putNumber("power wrist", speed);
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
