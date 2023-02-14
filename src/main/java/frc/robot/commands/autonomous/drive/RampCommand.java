// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
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
  double maxRoll = 0;
  double lockAngle = 0;
  PIDController pid = new PIDController(0.2, 0, 0);
  PIDController anglePid = new PIDController(0.02, 0, 0);
  public RampCommand(DriveSubsystem drive, VectorR vector) {
    this.drive = drive;
    this.vector = vector;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockAngle = DriveSubsystem.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double turnPower = MathR.limit(anglePid.calculate(DriveSubsystem.getYaw(), lockAngle), -0.5, 0.5);

    if (onRamp == false){
      drive.move(vector, turnPower);
    }
    else{
      double movement = MathR.limit((pid.calculate(DriveSubsystem.getRoll(), 0) * -1), -0.5, 0.5);
      drive.move(VectorR.fromPolar(movement, Math.PI/2), turnPower);
    }

    maxRoll = (DriveSubsystem.getRoll() <= maxRoll)?DriveSubsystem.getRoll():maxRoll;

    if (DriveSubsystem.getRoll() >= 20){
      climbingRamp = true;
      
    }
    
    if (DriveSubsystem.getRoll() >= maxRoll && climbingRamp){
      onRamp = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (DriveSubsystem.getRoll() >= -0.1 && DriveSubsystem.getRoll() <= 0.1 && onRamp){
      
      return true;
    }
    else{return false;}
    
  }
}
