// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class BalanceByOscillatingCommand extends DriveDirectionCommand {

  private final double startSpeed;
  private final Timer timer = new Timer();
  
  //Call this command after the robot has tipped over, front down, first move will be POSITIVE (away from drivers)
  public BalanceByOscillatingCommand(DriveSubsystem drive, double initialSpeed) {
    super(drive, VectorR.fromCartesian(initialSpeed, 0), 0);
    startSpeed = initialSpeed;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    super.initialize();
  }

  @Override
  public void execute() {
    double speed = -1 * 0.06 * DriveSubsystem.getRollDegrees()/(timer.get()*1.0);
    velocity.setMagnitude(MathR.limit(speed,-startSpeed, startSpeed));
    
    super.execute();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}