// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class DriveToTiltCommand extends DriveDirectionCommand {

  private final double tilt;
  private final boolean greaterThan;
  
  public DriveToTiltCommand(DriveSubsystem drive, VectorR velocity, double tilt, boolean greaterThan) {
    super(drive, velocity);
    this.tilt = tilt;
    this.greaterThan = greaterThan;
  }
  
  @Override
  public boolean isFinished() {
    return greaterThan ? DriveSubsystem.getRollDegrees() >= tilt : DriveSubsystem.getRollDegrees() <= tilt;
  }
}
