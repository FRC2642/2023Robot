// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos.real;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.claw.EndWhenObjectInClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DriveTowardsGamePieceCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetCarriageCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetShoulderCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BSHIGHCUBEAutoCommand extends SequentialCommandGroup {
  /** Creates a new TWOCUBESHOOTAutoCommand. */
  public BSHIGHCUBEAutoCommand(DriveSubsystem drive, LimelightSubsystem clawLimelight, CarriageSubsystem carriage, ShoulderSubsystem shoulder, ClawIntakeSubsystem intake, ClawGripperSubsystem gripper, SliderSubsystem sliders, ClawWristSubsystem wrist) {
    PiratePath path = new PiratePath("BSCUBE", false);
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths();

    var driveToCube = paths.get(0);
    var driveBackToPlace = paths.get(1);
    var driveToNextCube = paths.get(2);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(180),
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      
      new RunIntakeCommand(intake, 0.2).raceWith(new SetCarriageCommand(carriage, ()->CarriagePosition.EXTENDED)),
      new RunIntakeCommand(intake, -.2).withTimeout(0.3),

      new SetCarriageCommand(carriage, ()->CarriagePosition.RETRACTED).alongWith(
        new SetShoulderCommand(shoulder, () -> ShoulderPosition.PICKUP_GROUND),
      new FollowPathCommand(drive, driveToCube, true, 0)),

      new DriveTowardsGamePieceCommand(drive, clawLimelight, LimelightSubsystem.DetectionType.CUBE, 0.15).raceWith(new RunIntakeCommand(intake, 0.4), new EndWhenObjectInClawCommand(0.25)),
    
      new RunIntakeCommand(intake, 0.2).raceWith(new FollowPathCommand(drive, driveBackToPlace, false, 0.5).alongWith(new SetShoulderCommand(shoulder, () -> ShoulderPosition.PICKUP_HUMANPLAYER))),

      new RunIntakeCommand(intake, -0.2).withTimeout(0.5),
      new FollowPathCommand(drive, driveToNextCube, false, 0.5).alongWith(
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, sliders, carriage, wrist))
   
      );
  }
}
