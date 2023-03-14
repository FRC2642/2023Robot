// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.waiters.EndWhenObjectInClawCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeObjectCommand extends SequentialCommandGroup {
  /** Creates a new IntakeObjectCommand. */
  public IntakeObjectCommand(ClawIntakeSubsystem intake, ClawGripperSubsystem claw, LimelightSubsystem.DetectionType object) {
    
    addCommands(
      new OpenCloseClawCommand(claw, true),
      new RunIntakeCommand(intake, 0.5).raceWith(new EndWhenObjectInClawCommand(true))
    );
    if (object == DetectionType.CONE) addCommands(
      new OpenCloseClawCommand(claw, false),
      new RunIntakeCommand(intake, 0.5).raceWith(new WaitCommand(1)));
   
  }
}
