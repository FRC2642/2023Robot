// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriveToTiltCommand extends DriveDirectionCommand {

  private final double tilt;
  private final boolean greaterThan;

  private final double changeSpeedTime;
  private final double changeSpeed;

  private final double startSpeed;

  private final Timer timer = new Timer();
  
  public DriveToTiltCommand(DriveSubsystem drive, VectorR velocity, double tilt, boolean greaterThan, double changeSpeedTime, double changeSpeed) {
    super(drive, velocity, 180);
    startSpeed = velocity.getMagnitude();
    this.tilt = tilt;
    this.greaterThan = greaterThan;
    this.changeSpeedTime = changeSpeedTime;
    this.changeSpeed = changeSpeed;
  }
  
  public DriveToTiltCommand(DriveSubsystem drive, VectorR velocity, double tilt, boolean greaterThan) {
    super(drive, velocity, 180);
    startSpeed = velocity.getMagnitude();
    this.tilt = tilt;
    this.greaterThan = greaterThan;
    this.changeSpeedTime = 10;
    this.changeSpeed = startSpeed;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    super.initialize();
  }

  @Override
  public void execute() {
    velocity.setMagnitude(
      MathR.limit(
        MathR.lerp(startSpeed, changeSpeed, 0.0, changeSpeedTime, timer.get()), 
        changeSpeed, startSpeed));

    super.execute();
  }
  
  @Override
  public boolean isFinished() {
    return greaterThan ? DriveSubsystem.getRollDegrees() >= tilt : DriveSubsystem.getRollDegrees() <= tilt;
  }
}
