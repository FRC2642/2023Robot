// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.claw.EndWhenObjectInClawCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DriveFacingObjectCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BSBOTTOMLINKAutoCommand extends SequentialCommandGroup {
  /** Creates a new BSBOTTOMLINKAutoCommand. */
  public BSBOTTOMLINKAutoCommand(DriveSubsystem drive, ShoulderSubsystem shoulder, SliderSubsystem slider, CarriageSubsystem carriage, ClawIntakeSubsystem intake, ClawGripperSubsystem pneumatics, LimelightSubsystem camera) {
    PiratePath path = new PiratePath("2CUBESHOOT V1");
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths();

    var getFirstCube = paths.get(0);
    var shootFirstCube = paths.get(1);
    var getSecondCube = paths.get(2);
    var shootSecondCube = paths.get(3);
    var goToField = paths.get(4);

    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(180),
      new ResetSliderEncoderCommand(SliderPosition.RETRACTED),
      new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED),
      new ResetWristEncoderCommand(WristPosition.HORIZONTAL1),


      new OpenCloseClawCommand(pneumatics, true),
      new RunCommand(()->{
        shoulder.set(0.2);
      }, shoulder).withTimeout(0.8),

      new RunIntakeCommand(intake, 0.2).raceWith(
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_HUMAN_PLAYER, shoulder, slider, carriage)),
      new OpenCloseClawCommand(pneumatics, false).alongWith(
      new RunIntakeCommand(intake, -0.2).withTimeout(0.3)),

      new OpenCloseClawCommand(pneumatics, true),
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, slider, carriage).alongWith(
        new FollowPathCommand(drive, getFirstCube, true, 0)
      ),
      new DriveFacingObjectCommand(drive, camera, LimelightSubsystem.DetectionType.CUBE, VectorR.fromPolar(0.15, 0)).withTimeout(2).raceWith(
        new RunIntakeCommand(intake, 0.4)
      ).raceWith(
        new EndWhenObjectInClawCommand(0.5)
      ),
      new RunIntakeCommand(intake, 0.2).raceWith(new FollowPathCommand(drive, shootFirstCube, false, 0.5).alongWith(
        new SetRobotConfigurationCommand(RobotConfiguration.SHOOT_CUBE, shoulder, slider, carriage))
      ),
      new OpenCloseClawCommand(pneumatics, false).alongWith(
      new RunIntakeCommand(intake, -1).withTimeout(0.3)),

      new OpenCloseClawCommand(pneumatics, true),
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, slider, carriage).alongWith(
        new FollowPathCommand(drive, getSecondCube, false, 0.5)
      ),
      
      
      new DriveFacingObjectCommand(drive, camera, LimelightSubsystem.DetectionType.CUBE, VectorR.fromPolar(0.15, 40)).withTimeout(2).raceWith(new RunIntakeCommand(intake, 0.4)).raceWith(new EndWhenObjectInClawCommand(0.5)),
      

      new RunIntakeCommand(intake, 0.2).raceWith(new FollowPathCommand(drive, shootSecondCube, false, 0.5).alongWith(
        new SetRobotConfigurationCommand(RobotConfiguration.SHOOT_CUBE, shoulder, slider, carriage))
      ),
      
      new OpenCloseClawCommand(pneumatics, false).alongWith(
      new RunIntakeCommand(intake, -1).withTimeout(0.3)),

      new FollowPathCommand(drive, goToField, false, 0.5).alongWith(new SetRobotConfigurationCommand(RobotConfiguration.TRAVEL_MODE, shoulder, slider, carriage))
      

    );
  }
}
