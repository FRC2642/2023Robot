// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class FollowVectorCommand extends CommandBase {
  /** Creates a new FollowVectorCommand. */
  private DriveSubsystem drive;
  private VectorR velocity;
  private VectorR distance;
  private double faceDegree;
  public boolean set_period = false;
  private VectorR periodVector = new VectorR();
  private VectorR initVector = new VectorR();
  private PIDController autoPid = new PIDController(0.02, 0, 0);

  public FollowVectorCommand(DriveSubsystem drive, VectorR distance, VectorR velocity, double faceDegree) {
    //THIS COMMAND TAKES IN A VECTOR WITH AN ANGLE IN RADIANS AND AN ORIENTATION IN DEGREES
    this.drive = drive;
    this.velocity = velocity;
    this.distance = distance;
    this.faceDegree = faceDegree;
    set_period = false;
    addRequirements(drive);
  }

  public void setPeriodOrigin(){
    initVector = drive.getDisplacement().clone();
  }

  public void setPeriodVector(){
    periodVector = VectorR.subVectors(drive.getDisplacement().clone(), initVector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.resetDisplacement(VectorR.fromCartesian(0, 0));
    initVector.setFromCartesian(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (set_period == false){
      setPeriodOrigin();
      set_period = true;
    }

    double reference = 0;
    double turnWheelSpeed = 0;
      
    reference = MathR.halfOptimize(DriveSubsystem.getYawDegrees(), faceDegree, 360);
    
    //0.25 speed auto
    //turnWheelSpeed = MathR.limit(pid.calculate(reference, faceDegree), -1, 1) * 400;
    //0.5 speed auto
    //turnWheelSpeed = MathR.limit(pid.calculate(reference, faceDegree), -1, 1) * 100;
    //0.75 speed auto
    turnWheelSpeed = MathR.limit(autoPid.calculate(reference, faceDegree), -1, 1);
    drive.move(velocity, turnWheelSpeed);

    
    //profiler.updatePosition();
    setPeriodVector();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("pv " + periodVector.getMagnitude());
    System.out.println("dist " + distance.getMagnitude());
    System.out.println(VectorR.compareVectors(periodVector, distance));
    return VectorR.compareVectors(periodVector, distance);
  }
}
