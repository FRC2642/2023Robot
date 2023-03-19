// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.claw.IntakeObjectCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DriveFacingObjectCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetCarriageCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetShoulderCommand;
import frc.robot.commands.autonomous.positionable.SetSliderCommand;
import frc.robot.commands.autonomous.positionable.SetWristCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.game.GamePieceType;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.VectorR;

public class BSCUBEAutoCommand extends SequentialCommandGroup {
  
  public BSCUBEAutoCommand(DriveSubsystem drive, LimelightSubsystem camera, CarriageSubsystem carriage, SliderSubsystem sliders, ShoulderSubsystem shoulder, ClawWristSubsystem wrist, ClawIntakeSubsystem intake, ClawGripperSubsystem gripper) {
    PiratePath path = new PiratePath("BSCUBE");
    var subs = path.getSubPaths();
    PiratePath driveToCubePath = subs.get(0);
    PiratePath driveToBackToShelfPath = subs.get(1);
    PiratePath driveToSecondObjectPath = subs.get(2);

    addCommands(
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      new ResetWristEncoderCommand(WristPosition.HORIZONTAL1),
      new SetShoulderCommand(shoulder, () -> ShoulderPosition.PICKUP_GROUND).withTimeout(0.2),
      new SetRobotConfigurationCommand(() -> RobotConfiguration.PLACE_CONE_HIGH, shoulder, sliders, carriage),
      new OpenCloseClawCommand(gripper, true),
      new SetRobotConfigurationCommand(() -> RobotConfiguration.PICKUP_FLOOR, shoulder, sliders, carriage).alongWith(
        new WaitCommand(1.5).andThen(new FollowPathCommand(drive, driveToCubePath, true))
      ),
      new DriveFacingObjectCommand(drive, camera, VectorR.fromCartesian(0.3, 0.0)).raceWith(new IntakeObjectCommand(intake, gripper, GamePieceType.CUBE)),
      new SetShoulderCommand(shoulder, () -> ShoulderPosition.PLACE_CUBE_HIGH).alongWith(
        new FollowPathCommand(drive, driveToBackToShelfPath, false).raceWith(new RunIntakeCommand(intake, 0.1))
      ),
      new RunIntakeCommand(intake, -0.5).withTimeout(0.5),
      new WaitCommand(1),
      new FollowPathCommand(drive, driveToSecondObjectPath, false).alongWith(
        new WaitCommand(1).andThen(new SetShoulderCommand(shoulder, () -> ShoulderPosition.PICKUP_GROUND)))
      );
      //TESTABLE
  }
}
