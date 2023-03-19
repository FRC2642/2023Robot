// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.drive.DriveDistanceCommand;
import frc.robot.commands.autonomous.drive.DriveUpAndBalanceBackwardsCommand;
import frc.robot.commands.autonomous.drive.DriveUpAndBalanceCommand;
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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BALANCEAutoCommand extends SequentialCommandGroup {
  /** Creates a new ScoreHighTaxiBalanceAuto. */
  public BALANCEAutoCommand(SliderSubsystem sliders, ClawGripperSubsystem pneumatics, DriveSubsystem drive, CarriageSubsystem carriage, ShoulderSubsystem shoulder) {

    addCommands(
      new ResetGyroCommand(180),
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      new ResetWristEncoderCommand(WristPosition.HORIZONTAL1),
      new SetShoulderCommand(shoulder, () -> ShoulderPosition.PICKUP_GROUND).withTimeout(0.2),
      new SetRobotConfigurationCommand(() -> RobotConfiguration.PLACE_CONE_HIGH, shoulder, sliders, carriage),
      new OpenCloseClawCommand(pneumatics, true),
      new SetRobotConfigurationCommand(() -> RobotConfiguration.STARTING_CONFIG, shoulder, sliders, carriage),
      new DriveDistanceCommand(drive, VectorR.fromPolar(0.4, 0), 15),
      new DriveUpAndBalanceBackwardsCommand(drive)
    ); 
  }
}
