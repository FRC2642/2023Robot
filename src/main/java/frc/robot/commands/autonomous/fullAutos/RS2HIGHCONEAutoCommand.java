// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DivertToGamePieceCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
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
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RS2HIGHCONEAutoCommand extends SequentialCommandGroup {
  /** Creates a new RSHIGHCONEAutoCommand. */
  

  public RS2HIGHCONEAutoCommand(DriveSubsystem drive, LimelightSubsystem clawLimelight, ShoulderSubsystem shoulder, SliderSubsystem sliders, CarriageSubsystem carriage, ClawWristSubsystem wrist, ClawGripperSubsystem gripper, ClawIntakeSubsystem intake) {
    PiratePath path = new PiratePath("RS2HIGHCONE");
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths();
  
    var driveToCone = paths.get(0);
    var driveBackToPlace = paths.get(1);
    var driveToField = paths.get(2);
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(180),
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),

      new OpenCloseClawCommand(gripper, false),
      new RunCommand(()->{
        shoulder.set(0.7);
      }, shoulder).withTimeout(0.3),
      
      
      
      new SetRobotConfigurationCommand(RobotConfiguration.PLACE_CONE_HIGH_AUTO, shoulder, sliders, carriage, wrist).raceWith(new RunIntakeCommand(intake, 0.2)),
      new WaitCommand(0.2),
      new OpenCloseClawCommand(gripper, true),
      new WaitCommand(0.2),
      
      new OpenCloseClawCommand(gripper, false),
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, sliders, carriage, wrist).alongWith(
        new WaitCommand(0.5).andThen(
        new DivertToGamePieceCommand(drive, clawLimelight, LimelightSubsystem.DetectionType.CONE, driveToCone, true, 0, 0.20, 2, true).raceWith(
          new RunIntakeCommand(intake, 1)
      ))),
        

      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_HUMAN_PLAYER, shoulder, sliders, carriage, wrist).alongWith(
        new FollowPathCommand(drive, driveBackToPlace, false, 0.5)),
      
      
      new SetRobotConfigurationCommand(RobotConfiguration.PLACE_CONE_HIGH, shoulder, sliders, carriage, wrist),
      new WaitCommand(0.2),
      new OpenCloseClawCommand(gripper, true),
      new WaitCommand(0.2),
      new SetRobotConfigurationCommand(RobotConfiguration.TRAVEL_MODE, shoulder, sliders, carriage, wrist).alongWith(new FollowPathCommand(drive, driveToField, false, 0.5))
      
    
    

    );
  }
}
