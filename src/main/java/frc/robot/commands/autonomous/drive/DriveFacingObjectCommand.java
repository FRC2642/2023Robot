// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.VectorR;

public class DriveFacingObjectCommand extends DriveDirectionCommand {

  private final LimelightSubsystem camera;

  public DriveFacingObjectCommand(DriveSubsystem drive, LimelightSubsystem camera, VectorR velocity) {
    super(drive, velocity);
    this.camera = camera;
  }
  
  @Override
  public void execute() {
    if (camera.isDetection && camera.confidence() > 5)
      heading = DriveSubsystem.getYawDegrees() + camera.x;
    super.execute();
  }
}
