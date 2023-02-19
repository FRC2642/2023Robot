// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class ResetDisplacementCommand extends CommandBase {

  final DriveSubsystem drive;
  
  public ResetDisplacementCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    DriveSubsystem.resetDisplacement(new VectorR());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
