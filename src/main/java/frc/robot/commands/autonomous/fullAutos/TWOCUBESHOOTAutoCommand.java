// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TWOCUBESHOOTAutoCommand extends SequentialCommandGroup {
  /** Creates a new TWOCUBESHOOTAutoCommand. */
  public TWOCUBESHOOTAutoCommand(DriveSubsystem drive) {
    PiratePath path = new PiratePath("2CUBESHOOT V2");
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FollowPathCommand(drive, path, true, 0));
  }
}
