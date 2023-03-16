// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriveDirectionCommand extends CommandBase { 
  
  public static final double TURN_KP = 0.01;
  public static final double DRIVE_KP = 0.3;
  
  protected final DriveSubsystem drive;
  protected VectorR velocity;
  protected Double heading;

  protected final VectorR localDisplacement = new VectorR();

  public DriveDirectionCommand(DriveSubsystem drive, VectorR velocity, double heading) {
    this.drive = drive;
    this.velocity = velocity;
    this.heading = heading;
    addRequirements(drive);
  }
  public DriveDirectionCommand(DriveSubsystem drive, VectorR velocity) {
    this.drive = drive;
    this.velocity = velocity;
    heading = Double.NaN;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    localDisplacement.setFromCartesian(0, 0);
    if (heading == Double.NaN) heading = DriveSubsystem.getYawDegrees();
  }

  @Override
  public void execute() {
    localDisplacement.add(DriveSubsystem.getRelativeIncrement());

    double turnSpeed = TURN_KP * MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), heading);
  
    VectorR rotatedDisplacement = localDisplacement.clone();
    rotatedDisplacement.rotate(-velocity.getAngle());
    VectorR antiStrafe = VectorR.fromPolar(rotatedDisplacement.getY(), velocity.getAngle() - 90);
    antiStrafe.mult(DRIVE_KP);
    VectorR driveSpeed = VectorR.addVectors(velocity, antiStrafe);

    drive.move(driveSpeed,  MathR.limit(turnSpeed, -1, 1));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
