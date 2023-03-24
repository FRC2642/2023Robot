// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.VectorR;

public class TurnTowardsGamePieceCommand extends CommandBase {

  DriveSubsystem drive;
  XboxController mainControl;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType type;

  final VectorR leftJoystick = new VectorR();

  public TurnTowardsGamePieceCommand(DriveSubsystem drive, LimelightSubsystem limelight, LimelightSubsystem.DetectionType type, XboxController mainControl) {
    this.drive = drive;
    this.limelight = limelight;
    this.type = type;
    this.mainControl = mainControl;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    limelight.setDetectionType(type);
  }

  @Override
  public void execute() {
    leftJoystick.setFromCartesian(mainControl.getLeftX(), -mainControl.getLeftY());
    leftJoystick.rotate(-90);
    
    limelight.setDetectionType(type);

    SmartDashboard.putNumber("centerX", limelight.x);

    if (limelight.isDetection) drive.move(leftJoystick, limelight.x * -1 * (1d/37d));
    else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
