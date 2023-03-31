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
import frc.robot.utils.VectorR;
import frc.robot.path.*;

public class FollowPathCommand extends CommandBase {

  public static final double HEADING_KP = 0.00027925;
  public static final double MOVEMENT_KP = .016;
  public static final double BASE_PRECISION = 0.05;
  public static final double TIME_TO_CORRECT_FROM_START = 1.5;

  public double lookAheadTime = BASE_PRECISION;
  public final Double startingLookAheadTime;

  private final DriveSubsystem drive;
  private final Timer timer = new Timer();
  private final PiratePath notAdjustedPath;

  private Iterator<PiratePoint> iterator = null;
  private PiratePath path;
  private double currentTime;
  private final boolean recenterDisplacementToFirstPoint;

  public FollowPathCommand(DriveSubsystem drive, PiratePath path, boolean recenterDisplacementToFirstPoint, double additionalLookaheadTime) {
    this.drive = drive;
    notAdjustedPath = path;
    this.recenterDisplacementToFirstPoint = recenterDisplacementToFirstPoint;

    if (additionalLookaheadTime != 0.0) startingLookAheadTime = BASE_PRECISION + additionalLookaheadTime;
    else startingLookAheadTime = null;
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {

     if (notAdjustedPath != null && notAdjustedPath.allianceDependent && DriverStation.getAlliance() == Alliance.Blue)
       setPath(notAdjustedPath.getBlueAlliance());
     else
      setPath(notAdjustedPath);

    startPath();
  }

  protected void setPath(PiratePath path) {
    this.path = path;
  }

  protected boolean startPath() {
    if (path == null) {
      return false;
    }
    this.iterator = path.iterator();
    timer.reset();
    timer.start();
    currentTime = timer.get() + path.getFirst().time;
    if (recenterDisplacementToFirstPoint) {
      DriveSubsystem.resetDisplacement(path.getFirst().position);
      DriveSubsystem.resetGyro(path.getFirst().heading);
    }
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
    if (!isStarted()) {
      drive.stop();
      return;
    }

    if (startingLookAheadTime == null) lookAheadTime = BASE_PRECISION;
    else {
      lookAheadTime = MathR.lerp(startingLookAheadTime, BASE_PRECISION, 0.0, TIME_TO_CORRECT_FROM_START, timer.get());
      lookAheadTime = MathR.limit(lookAheadTime, BASE_PRECISION, startingLookAheadTime);
    }

    currentTime = timer.get() + path.getFirst().time;

    while ((nextPoint == null || nextPoint.time - currentTime < lookAheadTime) && iterator.hasNext())
      nextPoint = iterator.next();

    var delta_t = nextPoint.time - currentTime;
    if (delta_t < lookAheadTime) delta_t = lookAheadTime;
    
    var velocity = nextPoint.position.clone();
    velocity.sub(DriveSubsystem.getRelativeFieldPosition());
    velocity.mult(MOVEMENT_KP / delta_t);

    double turn = MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), nextPoint.heading) / delta_t;

    drive.move(velocity, turn * HEADING_KP);
  }

  @Override
  public boolean isFinished() {
    return currentTime > path.getLastTime();
  }
}
