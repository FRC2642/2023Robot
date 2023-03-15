// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class DriveUpAndBalanceCommand extends SequentialCommandGroup {
  public DriveUpAndBalanceCommand(DriveSubsystem drive) {
    addCommands(
      new DriveToTiltCommand(drive, VectorR.fromPolar(0.3, Math.PI), -10, false),
      new DriveToTiltCommand(drive, VectorR.fromPolar(0.15, Math.PI), -7, true),
      new WaitCommand(2),
      new DriveDistanceCommand(drive, VectorR.fromPolar(0.2, 0), 0.5)
    );
  }
}
