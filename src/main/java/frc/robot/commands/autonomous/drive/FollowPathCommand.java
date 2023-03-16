// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import java.util.Iterator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.path.*;

public class FollowPathCommand extends CommandBase {

  public static final double HEADING_KP = 0.00027925;
  public static final double MOVEMENT_KP = .016;
  public static final double PRECISION = 0.05;

  private final DriveSubsystem drive;
  private final Timer timer = new Timer();
  private final PiratePath notAdjustedPath;

  private Iterator<PiratePoint> iterator = null;
  private PiratePath path;
  private double currentTime;

  public FollowPathCommand(DriveSubsystem drive, PiratePath path) {
    this.drive = drive;
    notAdjustedPath = path;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    
    if (DriverStation.getAlliance() == Alliance.Blue && path.allianceDependent)
      setPath(notAdjustedPath.getBlueAlliance());
    else
      setPath(notAdjustedPath);

    startPath();
  }

  protected void setPath(PiratePath path) {
    this.path = path;
  }

  protected boolean startPath() {
    if (path == null)
      return false;
    this.iterator = path.iterator();
    timer.reset();
    timer.start();
    currentTime = timer.get() + path.getFirst().time;
    DriveSubsystem.resetDisplacement(path.getFirst().position);
    DriveSubsystem.resetGyro(Math.toDegrees(path.getFirst().heading));
    return true;
  }

  protected void stopAndResetPath() {
    iterator = null;
  }

  protected boolean isStarted() {
    return iterator != null;
  }

  PiratePoint nextPoint = null;

  @Override
  public void execute() {
    currentTime = timer.get() + path.getFirst().time;

    while ((nextPoint == null || nextPoint.time - currentTime < PRECISION) && iterator.hasNext())
      nextPoint = iterator.next();

    var delta_t = nextPoint.time - currentTime;

    var velocity = nextPoint.position.clone();
    velocity.sub(DriveSubsystem.getRelativeFieldPosition());
    velocity.mult(MOVEMENT_KP / delta_t);

    double turn = MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), nextPoint.heading) / delta_t;

    drive.move(velocity, turn * HEADING_KP);
  }

  @Override
  public boolean isFinished() {
    return timer.get() + path.getFirst().time > path.getLastTime();
  }
}
