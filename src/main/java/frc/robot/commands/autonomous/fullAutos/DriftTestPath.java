// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriftTestPath extends SequentialCommandGroup {
  /** Creates a new DriftTestPath. */
  public DriftTestPath(DriveSubsystem drive) {
    PiratePath path = new PiratePath("Drift Test Path");
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);

    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(180),
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      new FollowPathCommand(drive, path, true, 0.5)
    );
  }
}
