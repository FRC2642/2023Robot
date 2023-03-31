// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.drive.DriveFacingObjectCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BSHIGHCONEAutoCommand extends SequentialCommandGroup {
  /** Creates a new TWOCUBESHOOTAutoCommand. */
  public BSHIGHCONEAutoCommand(DriveSubsystem drive, LimelightSubsystem clawLimelight) {
    PiratePath path = new PiratePath("PICKUP");
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths();

    var driveToCube = paths.get(0);
    var driveBackToPlace = paths.get(1);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FollowPathCommand(drive, driveToCube, true, 0),
      new DriveFacingObjectCommand(drive, clawLimelight, LimelightSubsystem.DetectionType.CUBE, VectorR.fromPolar(0.15, 0)).withTimeout(2),
      new FollowPathCommand(drive, driveBackToPlace, false, 0.5)
      );
  }
}
