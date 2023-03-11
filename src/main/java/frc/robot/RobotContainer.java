package frc.robot;

import java.io.IOException;
import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.claw.ManageClawPneumaticCommand;
import frc.robot.commands.autonomous.claw.RunIntakeSecondsCommand;
import frc.robot.commands.autonomous.drive.DistanceRampCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.drive.FollowVectorCommand;
import frc.robot.commands.autonomous.drive.RampCommand;
import frc.robot.commands.autonomous.drive.RecenterDisplacementCommand;
import frc.robot.commands.autonomous.fullAutos.BalanceRampCommand;
import frc.robot.commands.autonomous.fullAutos.ScoreAndBalanceAuto;
import frc.robot.commands.autonomous.fullAutos.ScoreAndTaxiAuto;
import frc.robot.commands.autonomous.mast.SetSliderCommand;
import frc.robot.commands.autonomous.waiters.WaitFor;
import frc.robot.commands.teleop.ClawCommands.ClawPneumaticCommand;
import frc.robot.commands.teleop.ClawCommands.ClawWristCommand;
import frc.robot.commands.teleop.ClawCommands.ClawIntakeCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsVisionCommand;
import frc.robot.commands.teleop.MastCommands.MoveCarriageCommand;
import frc.robot.commands.teleop.MastCommands.MoveSliderCommand;
import frc.robot.commands.teleop.MastCommands.MoveShoulder;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyro;
import frc.robot.commands.teleop.resetters.ResetShoulderEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetTurnEncoderCommand;
import frc.robot.commands.teleop.resetters.ToggleProtectShoulder;
import frc.robot.commands.teleop.resetters.ToggleStopDefensivelyCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.path.*;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;

public class RobotContainer {

  
  private final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  private final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);

  private final DriveSubsystem drive = new DriveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ClawPneumaticSubsystem clawPneumatics = new ClawPneumaticSubsystem();
  public static final CarriageSubsystem carriage = new CarriageSubsystem();
  private final ClawIntakeSubsystem intake = new ClawIntakeSubsystem();
  private final SliderSubsystem slider = new SliderSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final ClawWristSubsystem wrist = new ClawWristSubsystem();

  PiratePath autoPath1;
  PiratePath autoPath2;
  PiratePath autoPath3;
  Command autonomous;

  //SendableChooser<Command> chooser = new SendableChooser<Command>();
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public RobotContainer() {

    Constants.AUTO_PATHS.add("1 left high cube and cone + balance.wpilib.json");
    Constants.AUTO_PATHS.add("1 right high cube + 2 right high cones.wpilib.json");
    Constants.AUTO_PATHS.add("2 right high and mid cubes.wpilib.json");
    Constants.AUTO_PATHS.add("2 center high cones + balance.wpilib.json");
    Constants.AUTO_PATHS.add("2 left high cubes + balance.wpilib.json");
    
    try {
      autoPath3 = new PiratePath(Constants.AUTO_PATHS.get(0), DriverStation.getAlliance() == Alliance.Red);
    } catch (JsonProcessingException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } 

    

    autoPath1 = new PiratePath();
    autoPath1.add(new PiratePoint(0, 0, 0, 0, false));
    autoPath1.add(new PiratePoint(-18, 0, 0, 8, false));
    autoPath1.fillWithSubPointsEasing(0.2, Functions.easeInOutCubic);

    
    autoPath2 = new PiratePath();
    autoPath2.add(new PiratePoint(0, 0, 0, 0, false));
    autoPath2.add(new PiratePoint(-8, 0, 0, 5, false));
    autoPath2.fillWithSubPointsEasing(0.2, Functions.easeInOutCubic);


    autoChooser.addOption("score and taxi right", new ScoreAndTaxiAuto(slider, clawPneumatics, drive, carriage, shoulder, autoPath1));
    autoChooser.addOption("score and taxi left", new ScoreAndTaxiAuto(slider, clawPneumatics, drive, carriage, shoulder, autoPath1));
    autoChooser.addOption("score and balance", new ScoreAndBalanceAuto(slider, clawPneumatics, drive, carriage, shoulder));
    autoChooser.addOption("move out of the way", new FollowPathCommand(drive, autoPath2));
    autoChooser.addOption("open claw", new ManageClawPneumaticCommand(clawPneumatics, true));
    autoChooser.addOption("follow vector", new FollowVectorCommand(drive, VectorR.fromPolar(10, 0), VectorR.fromPolar(0.3, 0), 0));
    autoChooser.addOption("balance", new BalanceRampCommand(drive));

    autoChooser.addOption("2 left high cubes + balance", new SequentialCommandGroup(
      new SetSliderCommand(slider, true)/*.alongWith(new SetCarriageCommand(carriage, true)).alongWith(new SetShoulderCommand(shoulder, 60 degrees))*/,
      new ManageClawPneumaticCommand(clawPneumatics, true),
      new SetSliderCommand(slider, false)/*.alongWith(new SetCarriageCommand(carriage, false)).alongWith(new SetShoulderCommand(shoulder, 80 degrees))*/,
      new FollowPathCommand(drive, autoPath1)/*.alongWith(new SetShoulderCommand(shoulder, 230 degrees)*/,
      new RunIntakeSecondsCommand(intake, 2, true)
      //NOT DONE
    ));
    //chooser.addOption("drive command", new JoystickOrientedDriveCommand(drive, auxControl));
    
    
    drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));//.alongWith(new RecenterDisplacementCommand(limelight)));
    clawPneumatics.setDefaultCommand(new ClawPneumaticCommand(clawPneumatics, mainControl, auxControl));
    carriage.setDefaultCommand(new MoveCarriageCommand(carriage, auxControl));
    intake.setDefaultCommand(new ClawIntakeCommand(intake, mainControl, auxControl));
    //slider.setDefaultCommand(new MoveSliderCommand(slider, auxControl));
    shoulder.setDefaultCommand(new MoveShoulder(shoulder, auxControl));
    wrist.setDefaultCommand(new ClawWristCommand(wrist, auxControl));
    

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(new ToggleProtectShoulder(shoulder));
    SmartDashboard.putData(new ResetWristEncoderCommand(wrist));
    SmartDashboard.putData(new ResetGyro(drive));
    SmartDashboard.putData(new ResetDisplacementCommand(drive));
    SmartDashboard.putData(new ResetShoulderEncoderCommand());
    

    

    new POVButton(mainControl, 0).whileTrue(new ResetGyro(drive));
    new POVButton(mainControl, 180).whileTrue(new RampCommand(drive, VectorR.fromCartesian(0, 0), true));
    new POVButton(mainControl, 270).whileTrue(new ToggleStopDefensivelyCommand(drive));
    new JoystickButton(mainControl, Button.kA.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.CONE));
    new JoystickButton(mainControl, Button.kB.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.FIDUCIAL));
    new JoystickButton(mainControl, Button.kX.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.CUBE));
    new JoystickButton(mainControl, Button.kY.value).whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.RETROREFLECTIVE));
    }
    
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}