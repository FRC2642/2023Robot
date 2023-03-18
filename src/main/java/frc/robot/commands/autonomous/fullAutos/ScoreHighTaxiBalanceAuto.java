// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.SetCarriageCommand;
import frc.robot.commands.autonomous.SetShoulderCommand;
import frc.robot.commands.autonomous.SetSliderCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.drive.DriveDistanceCommand;
import frc.robot.commands.autonomous.drive.DriveUpAndBalanceCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
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
public class ScoreHighTaxiBalanceAuto extends SequentialCommandGroup {
  /** Creates a new ScoreHighTaxiBalanceAuto. */
  public ScoreHighTaxiBalanceAuto(SliderSubsystem sliders, ClawGripperSubsystem pneumatics, DriveSubsystem drive, CarriageSubsystem carriage, ShoulderSubsystem shoulder, PiratePath path) {
    // Add your commands in the addCommands() call, e.g.
    
    addCommands(
      new SetSliderCommand(sliders, SliderPosition.EXTENDED).alongWith(new SetCarriageCommand(carriage, CarriagePosition.EXTENDED)).alongWith(new SetShoulderCommand(shoulder, ShoulderPosition.PLACE_CONE_HIGH)),
      new OpenCloseClawCommand(pneumatics, true),
      new SetSliderCommand(sliders, SliderPosition.RETRACTED),
      new SetShoulderCommand(shoulder, ShoulderPosition.STARTING_CONFIG).alongWith(new SetCarriageCommand(carriage, CarriagePosition.RETRACTED)),
      new DriveDistanceCommand(drive, VectorR.fromPolar(0.4, 0), 15),
      new DriveUpAndBalanceCommand(drive)
      //TESTABLE

    ); 
  }
}
