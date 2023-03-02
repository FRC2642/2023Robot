// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.VectorR;

public class TurnTowardsGamePieceCommand extends CommandBase {

  DriveSubsystem drive;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType type;

  /** Creates a new TurnTowardsGamePieceCommand. */
  public TurnTowardsGamePieceCommand(DriveSubsystem drive, LimelightSubsystem limelight, LimelightSubsystem.DetectionType type) {
    this.drive = drive;
    this.limelight = limelight;
    this.type = type;
    addRequirements(drive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setDetectionType(type);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(new VectorR(), limelight.x * -1 * (1d/37d));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
