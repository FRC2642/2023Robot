// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.drive.DriveDirectionCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.mast.SetCarriageCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighAndBalanceAuto extends SequentialCommandGroup {
  /** Creates a new ScoreHighAndBalanceAuto. */
  public ScoreHighAndBalanceAuto(DriveSubsystem drive, CarriageSubsystem carriage, ClawGripperSubsystem gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PiratePath path = null;
    try {
      path = new PiratePath("src/main/deploy/pathplanner/Backwards.path", false);
    } catch (Exception e) {
      e.printStackTrace();
    }
    addCommands(
      new SetCarriageCommand(carriage, CarriageSubsystem.CarriagePosition.EXTENDED),
      new OpenCloseClawCommand(gripper, true),
      new FollowPathCommand(drive, path)
    );
  }
}
