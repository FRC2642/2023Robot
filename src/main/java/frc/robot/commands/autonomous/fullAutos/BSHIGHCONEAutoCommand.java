// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.claw.OpenCloseClawCommand;
import frc.robot.commands.autonomous.claw.RunIntakeCommand;
import frc.robot.commands.autonomous.drive.DriveFacingObjectCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BSHIGHCONEAutoCommand extends SequentialCommandGroup {
  /** Creates a new TWOCUBESHOOTAutoCommand. */
  public BSHIGHCONEAutoCommand(DriveSubsystem drive, LimelightSubsystem clawLimelight, CarriageSubsystem carriage, ShoulderSubsystem shoulder, ClawIntakeSubsystem intake, ClawGripperSubsystem gripper, SliderSubsystem sliders) {
    PiratePath path = new PiratePath("BSCONE");
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths();

    var driveToCube = paths.get(0);
    var driveBackToPlace = paths.get(1);
    var driveToCone = paths.get(2);

    addCommands(
      new RunCommand(()->{
        shoulder.set(0.2);
      }, shoulder).withTimeout(0.5),
      new SetRobotConfigurationCommand(RobotConfiguration.PLACE_CONE_HIGH, shoulder, sliders, carriage),
      new OpenCloseClawCommand(gripper, true),
      new WaitCommand(1),
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, sliders, carriage).alongWith(
        new FollowPathCommand(drive, driveToCube, true, 0)),
      
      new DriveFacingObjectCommand(drive, clawLimelight, LimelightSubsystem.DetectionType.CUBE, VectorR.fromPolar(0.15, 0)).withTimeout(2).raceWith(new RunIntakeCommand(intake, 0.4)),
      
      new SetRobotConfigurationCommand(RobotConfiguration.TRAVEL_MODE, shoulder, sliders, carriage).alongWith(
        new FollowPathCommand(drive, driveBackToPlace, false, 0.5)),
      
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_HUMAN_PLAYER, shoulder, sliders, carriage),
      new RunIntakeCommand(intake, -0.5),
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, sliders, carriage).alongWith(
        new FollowPathCommand(drive, driveToCone, false, 0.5)
      )
    );
  }
}
