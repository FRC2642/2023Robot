// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveUpAndBalanceBackwardsCommand extends SequentialCommandGroup {
  /** Creates a new DriveUpAndBalanceBackwardsCommand. */
  public DriveUpAndBalanceBackwardsCommand(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToTiltCommand(drive, VectorR.fromPolar(0.3, 180), 10, true),
      new DriveToTiltCommand(drive, VectorR.fromPolar(0.15, 180), 7, false),
      new WaitCommand(2),
      new DriveDistanceCommand(drive, VectorR.fromPolar(0.2, 0), 0.5)
    );
  }
}
