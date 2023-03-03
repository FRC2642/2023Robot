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

public class RampCommand extends CommandBase {
  /** Creates a new RampCommand. */
  DriveSubsystem drive;
  VectorR vector;
  boolean onRamp = false;
  boolean climbingRamp = false;
  double maxTilt = 0;
  double lockAngle = 0;
  double maxSpeed = 0.2;
  Timer timer = new Timer();
  boolean timerStarted = false;
  PIDController pid = new PIDController(0.2, 0, 0);
  PIDController anglePid = new PIDController(0.02, 0, 0);
  public RampCommand(DriveSubsystem drive, VectorR vector, boolean onRamp) {
    this.drive = drive;
    this.vector = vector;
    this.onRamp = onRamp;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.resetGyro(0.0);
    lockAngle = DriveSubsystem.getYawDegrees();
    vector.mult(maxSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double turnPower = MathR.limit(anglePid.calculate(DriveSubsystem.getYawDegrees(), lockAngle), -0.5, 0.5);
    if (onRamp == false){
      drive.move(vector, turnPower);
    }
    else{
      double movement;
      
      if (vector.getAngle() == 0.0 || vector.getAngle() == Math.PI){
        movement = MathR.limit((pid.calculate(DriveSubsystem.getRoll(), 0) * 1), -0.10, 0.10);
      }
      else{
        movement = MathR.limit((pid.calculate(DriveSubsystem.getPitch(), 0) * 1), -0.10, 0.10);
      }
        drive.move(VectorR.fromPolar(movement, 0), turnPower);
    }

    if (vector.getAngle() == 0.0){
      maxTilt = (DriveSubsystem.getRoll() <= maxTilt)?DriveSubsystem.getRoll():maxTilt;
    }
    else if (vector.getAngle() == Math.PI){
      maxTilt = (DriveSubsystem.getRoll() >= maxTilt)?DriveSubsystem.getRoll():maxTilt;
    }
    else if (vector.getAngle() == 3*Math.PI/2){
      maxTilt = (DriveSubsystem.getPitch() >= maxTilt)?DriveSubsystem.getPitch():maxTilt;
    }
    else{
      maxTilt = (DriveSubsystem.getPitch() <= maxTilt)?DriveSubsystem.getPitch():maxTilt;
    }
    
    if (vector.getAngle() == 0.0){
      if (DriveSubsystem.getRoll() <= -15){
        climbingRamp = true;
      }
    }
    else if (vector.getAngle() == Math.PI){
      if (DriveSubsystem.getRoll() >= 15){
        climbingRamp = true;
      }
    }
    else if (vector.getAngle() == 3*Math.PI/2){
      if (DriveSubsystem.getPitch() >= 15){
        climbingRamp = true;
      }
    }
    else{
      if (DriveSubsystem.getPitch() <= -15){
        climbingRamp = true;
      }
    }
    
    if (vector.getAngle() == 0.0){
      if (DriveSubsystem.getRoll() >= maxTilt + 10 && climbingRamp){
        onRamp = true;
      }
    }
    else if (vector.getAngle() == Math.PI){
      if (DriveSubsystem.getRoll() <= maxTilt - 10 && climbingRamp){
        onRamp = true;
      }
    }
    else if (DriveSubsystem.getPitch() <= maxTilt - 10 && climbingRamp){
      if (DriveSubsystem.getPitch() <= maxTilt - 10 && climbingRamp){
        onRamp = true;
      }
    }
    else{
      if (DriveSubsystem.getPitch() >= maxTilt + 10 && climbingRamp){
        onRamp = true;
      }
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!timerStarted){
      timer.start();
    }
    if (vector.getAngle() == 0.0 || vector.getAngle() == Math.PI){
      if (DriveSubsystem.getRoll() >= -3 && DriveSubsystem.getRoll() <= 3 && onRamp && timer.get() >= 3){
        return true;
      }
      else if (DriveSubsystem.getRoll() >= -3 && DriveSubsystem.getRoll() <= 3 && onRamp){
        drive.stop();
        return false;
      }
      else{timer.reset(); timerStarted = false; return false;}
    }
    else{
      if (DriveSubsystem.getPitch() >= -3 && DriveSubsystem.getPitch() <= 3 && onRamp && timer.get() >= 3){
        return true;
      }
      else if (DriveSubsystem.getPitch() >= -3 && DriveSubsystem.getPitch() <= 3 && onRamp){
        drive.stop();
        return false;
      }
      else{timer.reset(); timerStarted = false; return false;}
    }
    
    
  }
}
