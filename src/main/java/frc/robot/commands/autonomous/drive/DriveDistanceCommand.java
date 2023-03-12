// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriveDistanceCommand extends DriveDirectionCommand {
  
  private final VectorR distance;
  
  public boolean set_period = false;
  private VectorR periodVector = new VectorR();
  private VectorR initVector = new VectorR();

  public DriveDistanceCommand(DriveSubsystem drive, VectorR distance, VectorR velocity, double faceDegree) {
    super(drive, velocity, faceDegree);
    this.distance = distance;
    set_period = false;
  }
  
  public void setPeriodOrigin(){
    initVector = DriveSubsystem.getRelativeFieldPosition();
  }

  public void setPeriodVector(){
    periodVector = VectorR.subVectors(DriveSubsystem.getRelativeFieldPosition(), initVector);
  }
  
  @Override
  public void initialize() {
    DriveSubsystem.resetDisplacement(VectorR.fromCartesian(0, 0));
    initVector.setFromCartesian(0, 0);
  }

  @Override
  public void execute() {
    
    if (set_period == false){
      setPeriodOrigin();
      set_period = true;
    }
    super.execute();
    
    //profiler.updatePosition();
    setPeriodVector();
  }
  
  @Override
  public boolean isFinished() {
    return periodVector.compare(distance, 0.2, Math.toRadians(5));
  }
}
