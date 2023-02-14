// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class FollowVectorCommand extends CommandBase {
  /** Creates a new FollowVectorCommand. */
  private DriveSubsystem drive;
  private VectorR vector;
  private double faceDegree;

  public FollowVectorCommand(DriveSubsystem drive, VectorR vector, double faceDegree) {
    //THIS COMMAND TAKES IN A VECTOR WITH AN ANGLE IN RADIANS AND AN ORIENTATION IN DEGREES
    this.drive = drive;
    this.vector = vector;
    this.faceDegree = faceDegree;
    drive.set_period = false;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.resetDisplacement(VectorR.fromCartesian(0, 0));
    drive.resetInitVector();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.rotationMove(vector, faceDegree);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return VectorR.compareVectors(drive.getPeriodVector(), vector);
  }
}
