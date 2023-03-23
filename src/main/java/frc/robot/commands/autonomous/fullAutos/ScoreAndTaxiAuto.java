// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetCarriageCommand;
import frc.robot.commands.autonomous.positionable.SetShoulderCommand;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndTaxiAuto extends SequentialCommandGroup {
  /** Creates a new ScoreBalance. */
  public ScoreAndTaxiAuto(SliderSubsystem sliders, ClawGripperSubsystem pneumatics, DriveSubsystem drive, CarriageSubsystem carriage, ShoulderSubsystem shoulder, ClawIntakeSubsystem intake, PiratePath path) {
    addCommands(
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      new ResetWristEncoderCommand(WristPosition.HORIZONTAL1),
      new RunIntakeCommand(intake, 0.2).raceWith(new SetCarriageCommand(carriage, ()->CarriagePosition.EXTENDED)),
      new RunIntakeCommand(intake, -.2).withTimeout(1),
      new SetCarriageCommand(carriage, ()->CarriagePosition.RETRACTED),
      //new OpenCloseClawCommand(pneumatics, true),
      //new WaitCommand(2),
      new FollowPathCommand(drive, new PiratePath("TaxiPath Copy"), true, 0.0).alongWith(
        new SetShoulderCommand(shoulder, () -> ShoulderPosition.PICKUP_GROUND))
       // new RunIntakeCommand(intake, 0.1))
      //new CarriageAutoCommand(carriage, encoderTick).alongWith(new SetSliderCommand(sliders, true)),
      //new ShoulderAutoCommand(shoulder, encoderTick),
      //new ManageClawPneumaticCommand(pneumatics, true),
      //new ShoulderAutoCommand(shoulder, encoderTick),
      //new CarriageAutoCommand(carriage, encoderTick).alongWith(new SetSliderCommand(sliders, false))
      //new FollowPathCommand(drive, path),
      //new RampCommand(drive, VectorR.fromPolar(1, 0), false)

    );
  }
}
