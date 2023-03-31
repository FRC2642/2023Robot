// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriveFacingObjectCommand extends DriveDirectionCommand {

  private final LimelightSubsystem camera;
  private final LimelightSubsystem.DetectionType objectType;

  public DriveFacingObjectCommand(DriveSubsystem drive, LimelightSubsystem camera, LimelightSubsystem.DetectionType objectType, VectorR velocity) {
    super(drive, velocity);
    this.camera = camera;
    this.objectType = objectType;
  }
  
  @Override
  public void execute() {
    camera.setDetectionType(objectType);

    if (camera.isDetection && camera.confidence() > 0.5)
      turnSpeed = MathR.limit(camera.x * (-1d/65d), -0.25, 0.25);
    
    super.execute();
  }
}
