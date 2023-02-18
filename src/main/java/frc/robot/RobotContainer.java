package frc.robot;


import java.io.IOException;
import java.util.ArrayList;

import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.FollowPathVisionRecenterCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.drive.RecenterDisplacementCommand;
import frc.robot.commands.autonomous.waiters.WaitFor;
import frc.robot.commands.teleop.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.JoystickTurnSpeedDriveCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyro;
import frc.robot.commands.teleop.resetters.ToggleStopDefensivelyCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.path.*;

public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final XboxController control = new XboxController(Constants.DRIVE_CONTROL_PORT);

  PiratePath testPath;
  Command testPathFollowCommand;

  public RobotContainer() {

    try {
      testPath = new PiratePath(Constants.TEST_PATH);
    } catch (JsonProcessingException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    var subs = testPath.getSubPaths();
    
    /*testPathFollowCommand = new SequentialCommandGroup(
      new FollowPathVisionRecenterCommand(new RecenterDisplacementCommand(limelight), new FollowPathCommand(drive, subs.get(0))),
      new WaitFor(drive, 2),
      new FollowPathVisionRecenterCommand(new RecenterDisplacementCommand(limelight), new FollowPathCommand(drive, subs.get(1))),
      new WaitFor(drive, 2),
      new FollowPathCommand(drive, subs.get(2))
    );*/

    testPathFollowCommand = new SequentialCommandGroup(
      new FollowPathCommand(drive, subs.get(0)),
      new WaitFor(drive, 2),
      new FollowPathCommand(drive, subs.get(1)),
      new WaitFor(drive, 2),
      new FollowPathCommand(drive, subs.get(2))
    );


    
    drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, control).alongWith(new RecenterDisplacementCommand(limelight)));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    SmartDashboard.putData(new ResetGyro(drive));
    SmartDashboard.putData(new ResetDisplacementCommand(drive));

    new POVButton(control, 0).whileTrue(new ResetGyro(drive));
    new POVButton(control, 270).whileTrue(new ToggleStopDefensivelyCommand(drive));

  }

  public Command getAutonomousCommand() {
    return testPathFollowCommand;// FollowPathVisionRecenterCommand(new RecenterDisplacementCommand(limelight), testPathFollowCommand);

  }
}