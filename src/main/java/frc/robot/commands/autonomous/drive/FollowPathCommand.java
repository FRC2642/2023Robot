// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import java.util.Iterator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.path.*;


public class FollowPathCommand extends CommandBase {

  final double HEADING_KP = .016;
  final double MOVEMENT_KP = .016;
  final double PRECISION = 0.05;

  private final DriveSubsystem drive;
  private final PiratePath path;
  private final Timer timer;

  final Iterator<PiratePoint> iterator;

  double time = 0.0;


  /** Creates a new FollowPathCommand. */
  public FollowPathCommand(DriveSubsystem drive, PiratePath path) {
    this.path = path;
    this.drive = drive;
    timer = new Timer();
    iterator = path.iterator();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    time = timer.get();
    time+=path.getFirst().time;
    DriveSubsystem.resetDisplacement(path.getFirst().position);
    DriveSubsystem.resetGyro(Math.toDegrees(path.getFirst().heading));

  }
  
  PiratePoint nextPoint = null;

  @Override
  public void execute() {
    
    time = timer.get() + path.getFirst().time;
    //while ((nextPoint == null || nextPoint.time - timer.get() < PRECISION) && iterator.hasNext()) nextPoint = iterator.next();
    while ((nextPoint == null || nextPoint.time - time < PRECISION) && iterator.hasNext()) nextPoint = iterator.next();
    
    var velocity = nextPoint.position.clone();
    velocity.sub(DriveSubsystem.getRelativeFieldPosition());

    //var delta_t = nextPoint.time - timer.get();
    var delta_t = nextPoint.time - time;

    velocity.mult(MOVEMENT_KP/delta_t);

    double turn = MathR.getDistanceToAngleRadians(Math.toRadians(DriveSubsystem.getYawDegrees()), nextPoint.heading)/ delta_t;

    drive.move(velocity, turn * HEADING_KP);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return time > (path.getDuration());
    //return timer.get() > (path.getDuration());
  }
}
