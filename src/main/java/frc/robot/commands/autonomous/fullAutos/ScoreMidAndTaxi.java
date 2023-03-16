// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.mast.SetCarriageCommand;
import frc.robot.commands.autonomous.mast.SetShoulderCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMidAndTaxi extends SequentialCommandGroup {
  
  public ScoreMidAndTaxi(DriveSubsystem drive, ShoulderSubsystem shoulder, CarriageSubsystem carriage, ClawGripperSubsystem claw, ClawIntakeSubsystem intake) {
   
    PiratePath path = null;
    try {
      path = new PiratePath("");
    } catch (IOException e) {
      e.printStackTrace();
    }

    addCommands(
      new SetShoulderCommand(shoulder, ShoulderPosition.PLACE_CONE_OFFSIDE)
      .alongWith(new SetCarriageCommand(carriage, CarriagePosition.EXTENDED)),
      new OpenCloseClawCommand(claw, true),
      new RunIntakeCommand(intake, -0.5).raceWith(new WaitCommand(0.5)),
      new FollowPathCommand(drive, path)
    );
  }
}
