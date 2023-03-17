// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.SetCarriageCommand;
import frc.robot.commands.autonomous.SetShoulderCommand;
import frc.robot.commands.autonomous.SetSliderCommand;
import frc.robot.commands.autonomous.SetWristCommand;
import frc.robot.commands.autonomous.claw.IntakeObjectCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DriveFacingObjectCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
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

public class HSCUBEAutoCommand extends SequentialCommandGroup {
  
  public HSCUBEAutoCommand(DriveSubsystem drive, LimelightSubsystem camera, CarriageSubsystem carriage, SliderSubsystem sliders, ShoulderSubsystem shoulder, ClawWristSubsystem wrist, ClawIntakeSubsystem intake, ClawGripperSubsystem gripper) {
    PiratePath path = new PiratePath("HSCUBE");
    var subs = path.getSubPaths();
    PiratePath driveToBumpPath = subs.get(0);
    PiratePath driveToCubePath = subs.get(1);
    PiratePath driveBackToBumpPath = subs.get(2);
    PiratePath driveBackOverBumpPath = subs.get(3);
    PiratePath driveBackToShelfPath = subs.get(4);
    PiratePath turnAroundPath = subs.get(5);


    addCommands(
      new SetCarriageCommand(carriage, CarriagePosition.EXTENDED),
      new SetShoulderCommand(shoulder, ShoulderPosition.PLACE_CONE_OFFSIDE),
      new SetSliderCommand(sliders, SliderPosition.EXTENDED),
      new OpenCloseClawCommand(gripper, true),
      new WaitCommand(1),
      new SetSliderCommand(sliders, SliderPosition.RETRACTED),
      new SetWristCommand(wrist, WristPosition.VERTICAL1),
      new SetCarriageCommand(carriage, CarriagePosition.RETRACTED),
      new WaitCommand(1),
      new FollowPathCommand(drive, driveToBumpPath, true).alongWith(new SetShoulderCommand(shoulder, ShoulderPosition.PICKUP_GROUND)),
      new FollowPathCommand(drive, driveToCubePath, false),
//NOT FINISHED

    );
     
  }
}
