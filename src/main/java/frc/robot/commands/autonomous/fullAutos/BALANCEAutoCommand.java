// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DriveDistanceCommand;
import frc.robot.commands.autonomous.drive.DriveToTiltCommand;
import frc.robot.commands.autonomous.drive.DriveUpAndBalanceBackwardsCommand;
import frc.robot.commands.autonomous.drive.DriveUpAndBalanceCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetCarriageCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetShoulderCommand;
import frc.robot.commands.autonomous.positionable.SetSliderCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.subsystems.DriveSubsystem;
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
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BALANCEAutoCommand extends SequentialCommandGroup {
  /** Creates a new ScoreHighTaxiBalanceAuto. */
  public BALANCEAutoCommand(SliderSubsystem sliders, ClawGripperSubsystem pneumatics, DriveSubsystem drive, CarriageSubsystem carriage, ShoulderSubsystem shoulder, ClawIntakeSubsystem intake) {

    PiratePath path = new PiratePath();
    path.add(new PiratePoint(0, 0, 180, 0, false));
    path.add(new PiratePoint(2, 0, 180, 0.9, true));
    path.fillWithSubPointsEasing(0.05, Functions.easeOutExpo);

    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(180),
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      
      new RunIntakeCommand(intake, 0.2).raceWith(new SetCarriageCommand(carriage, ()->CarriagePosition.EXTENDED)),
      new RunIntakeCommand(intake, -.3).withTimeout(1),
      new SetCarriageCommand(carriage, ()->CarriagePosition.RETRACTED).alongWith(
        new DriveToTiltCommand(drive, VectorR.fromPolar(0.35, 0), -10, false).andThen(
        new DriveToTiltCommand(drive, VectorR.fromPolar(0.35, 0), 10, true, 2, 0.3),
        new DriveToTiltCommand(drive, VectorR.fromPolar(0.1, 0), 2, false, 2, 0.35),
       
       // new DriveDistanceCommand(drive, VectorR.fromPolar(0.2, 0), 1.5),
        new FollowPathCommand(drive, path, true, 0.0),
        new DriveToTiltCommand(drive, VectorR.fromPolar(0.0, 180), 10, true, 3, 0.35),
        new DriveToTiltCommand(drive, VectorR.fromPolar(0.2, 180), 7, false ,3, 0.15),
        new RunCommand(() -> drive.stop(), drive).withTimeout(0.5),
        new DriveDistanceCommand(drive, VectorR.fromPolar(0.125, 0), 0.48))
      ),
      new RunCommand(() -> {
        drive.setDefensiveMode(true);
        drive.stop();
      }, drive)

       
     // new DriveUpAndBalanceBackwardsCommand(drive) 
    ); 
  }
}
