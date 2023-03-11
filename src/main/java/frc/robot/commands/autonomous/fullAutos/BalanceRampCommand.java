// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.drive.DistanceRampCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.drive.FollowVectorCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceRampCommand extends SequentialCommandGroup {
  /** Creates a new BalanceRampCommand. */
  public BalanceRampCommand(DriveSubsystem drive) {
    PiratePath path = new PiratePath();
    path.add(new PiratePoint(0, 0, 0, 0, false));
    path.add(new PiratePoint(1.5, 0, 0, 5, false));
    path.fillWithSubPointsEasing(.01, Functions.easeLinear);

    addCommands(
    new DistanceRampCommand(drive, VectorR.fromPolar(0.2, Math.PI)),
    new FollowVectorCommand(drive, VectorR.fromPolar(2, 0), 0)
    
    );
  }
}
