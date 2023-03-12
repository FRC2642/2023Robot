// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class DriveToTiltCommand extends DriveDirectionCommand {

  private final double tilt;
  private final boolean greaterThan;
  /** Creates a new DriveToTiltCommand. */
  public DriveToTiltCommand(DriveSubsystem drive, VectorR velocity, double faceDegree, double tilt, boolean greaterThan) {
    super(drive, velocity, faceDegree);
    this.tilt = tilt;
    this.greaterThan = greaterThan;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return greaterThan ? DriveSubsystem.getRollDegrees() >= tilt : DriveSubsystem.getRollDegrees() <= tilt;
  }
}
