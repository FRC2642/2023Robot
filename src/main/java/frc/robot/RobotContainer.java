package frc.robot;

import java.io.IOException;
import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.drive.RampCommand;
import frc.robot.commands.autonomous.drive.RecenterDisplacementCommand;
import frc.robot.commands.autonomous.waiters.WaitFor;
import frc.robot.commands.teleop.ClawCommands.ClawPneumaticCommand;
import frc.robot.commands.teleop.ClawCommands.ClawWristCommand;
import frc.robot.commands.teleop.ClawCommands.ClawIntakeCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsVisionCommand;
import frc.robot.commands.teleop.MastCommands.MoveCarriageCommand;
import frc.robot.commands.teleop.MastCommands.MoveSliderCommand;
import frc.robot.commands.teleop.MastCommands.MoveShoulder;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyro;
import frc.robot.commands.teleop.resetters.ResetTurnEncoderCommand;
import frc.robot.commands.teleop.resetters.ToggleStopDefensivelyCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.utils.VectorR;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.path.*;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;

public class RobotContainer {

  
  private final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  private final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);

  private final DriveSubsystem drive = new DriveSubsystem();
  //private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ClawPneumaticSubsystem clawPneumatics = new ClawPneumaticSubsystem();
  public static final CarriageSubsystem carriage = new CarriageSubsystem();
  private final ClawIntakeSubsystem intake = new ClawIntakeSubsystem();
  private final SliderSubsystem slider = new SliderSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final ClawWristSubsystem wrist = new ClawWristSubsystem();

  PiratePath testPath;
  Command testPathFollowCommand;

  SendableChooser<Command> chooser = new SendableChooser<Command>();
  public RobotContainer() {

    try {
      testPath = new PiratePath(Constants.TEST_PATH, true);
    } catch (JsonProcessingException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    var subs = testPath.getSubPaths();

    
    chooser.addOption("drive command", new JoystickOrientedDriveCommand(drive, auxControl));
    

    testPathFollowCommand = new SequentialCommandGroup(
      /*new FollowPathCommand(drive, subs.get(0)),
      new WaitFor(drive, 2),
      new FollowPathCommand(drive, subs.get(1)),
      new WaitFor(drive, 2),
      new FollowPathCommand(drive, subs.get(2))*/
      new FollowPathCommand(drive, testPath)
    );
    
    drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));//.alongWith(new RecenterDisplacementCommand(limelight)));
    clawPneumatics.setDefaultCommand(new ClawPneumaticCommand(clawPneumatics, mainControl, auxControl));
    carriage.setDefaultCommand(new MoveCarriageCommand(carriage, auxControl));
    intake.setDefaultCommand(new ClawIntakeCommand(intake, mainControl, auxControl));
    slider.setDefaultCommand(new MoveSliderCommand(slider, auxControl));
    shoulder.setDefaultCommand(new MoveShoulder(shoulder, auxControl));
    wrist.setDefaultCommand(new ClawWristCommand(wrist, auxControl));
    

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    SmartDashboard.putData(new ResetTurnEncoderCommand(drive));
    SmartDashboard.putData(new ResetGyro(drive));
    SmartDashboard.putData(new ResetDisplacementCommand(drive));
    

    new POVButton(mainControl, 0).whileTrue(new ResetGyro(drive));
    new POVButton(mainControl, 180).whileTrue(new RampCommand(drive, VectorR.fromCartesian(0, 0), true));
    new POVButton(mainControl, 270).whileTrue(new ToggleStopDefensivelyCommand(drive));
    //new JoystickButton(mainControl, Button.kA.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.CONE));
    //new JoystickButton(mainControl, Button.kB.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.FIDUCIAL));
    //new JoystickButton(mainControl, Button.kX.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.CUBE));
    //new JoystickButton(mainControl, Button.kY.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.RETROREFLECTIVE));
    }
    
  public Command getAutonomousCommand() {
    return testPathFollowCommand;
  }
}