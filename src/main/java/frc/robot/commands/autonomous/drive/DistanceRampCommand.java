// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DistanceRampCommand extends CommandBase {
  /** Creates a new DistanceRampCommand. */
  DriveSubsystem drive;
  boolean onRamp = false;
  boolean climbingRamp = false;
  
  double maxTilt = 0;
  double lockAngle = 0;
  VectorR vector;
  boolean finished = false;
  int tippingAmount = 2;

  Timer timer = new Timer();
  
  PIDController anglePid = new PIDController(0.02, 0, 0);
  public DistanceRampCommand(DriveSubsystem drive, VectorR velocity) {
    this.drive = drive;
    this.vector = velocity;
    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.calibrateGyro();
    DriveSubsystem.resetGyro(0.0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= 1){
      vector.div(timer.get());
    }
    double turnPower = MathR.limit(anglePid.calculate(DriveSubsystem.getYawDegrees(), lockAngle), -0.5, 0.5);
    if (onRamp == false){
      drive.move(vector, turnPower);
    }
    else{
      finished = true;
    }
    /*else{
      if (tippingForward){
        
      }
      else if (tippingBackward){

      }
      else{
        drive.stop();
      }
    }*/

    //maxTilt = (DriveSubsystem.getRoll() <= maxTilt)?DriveSubsystem.getRoll():maxTilt;
    if (DriveSubsystem.getRoll() <= -10){
      climbingRamp = true;
    }
    if (climbingRamp && DriveSubsystem.getRoll() >= 0){
      onRamp = true;
      
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
