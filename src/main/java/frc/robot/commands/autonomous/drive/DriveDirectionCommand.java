// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriveDirectionCommand extends CommandBase { 
  
  public static final double TURN_KP = 1.0;
  
  private final DriveSubsystem drive;
  private final VectorR velocity;
  private final double faceDegree;

  public DriveDirectionCommand(DriveSubsystem drive, VectorR velocity, double faceDegree) {
    //THIS COMMAND TAKES IN A VECTOR WITH AN ANGLE IN RADIANS AND AN ORIENTATION IN DEGREES
    this.drive = drive;
    this.velocity = velocity;
    this.faceDegree = faceDegree;
    addRequirements(drive);
  }

  @Override
  public void execute() {

    double reference = Math.toDegrees(MathR.getDistanceToAngleRadians(Math.toRadians(DriveSubsystem.getYawDegrees()), Math.toRadians(faceDegree)));
    double turnSpeed = MathR.limit(reference, -1, 1);
    
    drive.move(velocity, turnSpeed);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
