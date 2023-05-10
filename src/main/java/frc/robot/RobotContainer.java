package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.fullAutos.BSBOTTOMLINKAutoCommand;
import frc.robot.commands.autonomous.fullAutos.BSHIGHCONEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.CONEBALANCEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.RS2HIGHCONEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.RSBOTTOMLINKAutoCommand;
import frc.robot.commands.autonomous.fullAutos.real.CUBEBALANCEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.real.BSHIGHCUBEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.real.RSHIGHCUBEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.real.ScoreAndTaxiAuto;
import frc.robot.commands.autonomous.positionable.SetLEDsCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.autonomous.positionable.SetShoulderCommand;
import frc.robot.commands.autonomous.positionable.SetSliderCommand;
import frc.robot.commands.autonomous.positionable.SetWristCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopGripperCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopIntakeCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopWristCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsGamePieceCommand;
import frc.robot.commands.teleop.MastCommands.TeleopCarriageCommand;
import frc.robot.commands.teleop.MastCommands.TeleopShoulderCommand;
import frc.robot.commands.teleop.MastCommands.TeleopSliderCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.LEDs.LEDPattern;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;


public class RobotContainer {
  public final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  public final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  public final DriveSubsystem drive = new DriveSubsystem();
  public final LimelightSubsystem clawLimelight = new LimelightSubsystem("limelight-back");
  public final LimelightSubsystem poleLimelight = new LimelightSubsystem("limelight-front");

  public final ClawGripperSubsystem gripper = new ClawGripperSubsystem();
  public final CarriageSubsystem carriage = new CarriageSubsystem();
  public final ClawIntakeSubsystem intake = new ClawIntakeSubsystem();
  public final SliderSubsystem slider = new SliderSubsystem();
  public final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  public final ClawWristSubsystem wrist = new ClawWristSubsystem();
  //public final LEDs leds = new LEDs();


  public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public final PiratePath taxiPath = new PiratePath(false);

  public static boolean DEBUG = false;


  public RobotContainer() {

    taxiPath.add(new PiratePoint(0, 0, 180, 0, false));
    taxiPath.add(new PiratePoint(14.5, 0, 180, 8, false));
    taxiPath.fillWithSubPointsEasing(0.01, Functions.easeInOutCubic);

    SmartDashboard.putNumber("DEBUG MODE", 0);

    // Default commands

    // Auto options
    autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(5));
    autoChooser.addOption("Mobility High Cube",
        new ScoreAndTaxiAuto(slider, gripper, drive, carriage, shoulder, intake, taxiPath));
    autoChooser.addOption("Flat Side High Cone High Cube", new BSHIGHCONEAutoCommand(drive, clawLimelight, poleLimelight, carriage, shoulder, intake, gripper, slider, wrist));
    autoChooser.addOption("Bump Side High Cube Mid Cube", new RSHIGHCUBEAutoCommand(drive, clawLimelight, carriage, shoulder, intake, gripper, slider, wrist));
    autoChooser.addOption("Flat Side High Cube Mid Cube", new BSHIGHCUBEAutoCommand(drive, clawLimelight, carriage, shoulder, intake, gripper, slider, wrist));
    autoChooser.addOption("Balance High Cube", new CUBEBALANCEAutoCommand(slider, gripper, drive, carriage, intake));
    autoChooser.addOption("Flat Side 3 Low Cube", new BSBOTTOMLINKAutoCommand(drive, shoulder, slider, carriage, intake, gripper, wrist, clawLimelight));
    autoChooser.addOption("Bump Side 3 Low Cube", new RSBOTTOMLINKAutoCommand(drive, shoulder, slider, carriage, intake, gripper, wrist, clawLimelight));
    autoChooser.addOption("Balance High Cone", new CONEBALANCEAutoCommand(slider, gripper, drive, carriage, intake, shoulder, wrist)); 
    autoChooser.addOption("Bump Side 2 High Cone", new RS2HIGHCONEAutoCommand(drive, clawLimelight, shoulder, slider, carriage, wrist, gripper, intake)); 

    SmartDashboard.putData(autoChooser);

    
    // SmartDashboard.putData(new ResetDisplacementCommand(new VectorR()));
    SmartDashboard.putData(new ResetSliderEncoderCommand(SliderPosition.ZERO));
    SmartDashboard.putData(new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED));

  }

  public void autonomousInit() {
    drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
    carriage.setDefaultCommand(new RunCommand(() -> carriage.set(0.0), carriage));
    slider.setDefaultCommand(new RunCommand(() -> slider.set(0.0), slider));
    wrist.setDefaultCommand(new RunCommand(() -> wrist.set(), wrist));
    shoulder.setDefaultCommand(new RunCommand(() -> shoulder.set(0.0), shoulder));
    intake.setDefaultCommand(new RunCommand(() -> intake.set(0.0), intake));
  }

  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    SmartDashboard.putData("Starting Config", new SetShoulderCommand(shoulder, () -> ShoulderPosition.TRAVEL_MODE));

    if (!DEBUG) {
      drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));
      gripper.setDefaultCommand(new TeleopGripperCommand(gripper, auxControl));
      carriage.setDefaultCommand(new TeleopCarriageCommand(carriage,  auxControl));
      intake.setDefaultCommand(new TeleopIntakeCommand(intake, auxControl));
      slider.setDefaultCommand(new TeleopSliderCommand(slider, auxControl));
      shoulder.setDefaultCommand(new TeleopShoulderCommand(shoulder, auxControl));
      wrist.setDefaultCommand(new TeleopWristCommand(wrist, auxControl));
      //leds.setDefaultCommand(new SetLEDsCommand(leds, mainControl, auxControl));

      // BUTTONS

      //Fully extend button
      new JoystickButton(auxControl, 1).onTrue(
          (new SetRobotConfigurationCommand(RobotConfiguration.PLACE_CONE_HIGH, shoulder, slider, carriage, wrist)));
      //Fully retract button
      new JoystickButton(auxControl, 2).onTrue(
          (new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, slider, carriage, wrist)));
      //Override button
      new JoystickButton(auxButtonBoard, 6).onTrue(new InstantCommand(() -> {}, shoulder, wrist));
      //Chute button
      new JoystickButton(auxButtonBoard, 7).onTrue((new SetRobotConfigurationCommand(RobotConfiguration.CHUTE, shoulder, slider, carriage, wrist)));
      //Human button
      new JoystickButton(auxButtonBoard, 8)
          .onTrue((new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_HUMAN_PLAYER, shoulder, slider, carriage, wrist)));
      //Knocked down cone placement button
      new JoystickButton(auxButtonBoard, 9)
          .onTrue((new SetRobotConfigurationCommand(RobotConfiguration.PLACE_KOCKED_CONE, shoulder, slider, carriage, wrist)));
      //Travel button
      new JoystickButton(auxButtonBoard, 1)
        .onTrue((new SetRobotConfigurationCommand(RobotConfiguration.TRAVEL_MODE, shoulder, slider, carriage, wrist)));
      //Extend slider button
      new JoystickButton(auxButtonBoard, 12).onTrue(new SetSliderCommand(slider, () -> {
        SliderSubsystem.protectionEnabled = false;
        return SliderPosition.EXTENDED;
      }));
      //Retract slider button
      new JoystickButton(auxButtonBoard, 11).onTrue(new SetSliderCommand(slider, () -> {
        SliderSubsystem.protectionEnabled = false;
        return SliderPosition.RETRACTED;
      }));
      //Activate slider protection
      new JoystickButton(auxButtonBoard, 10).onTrue(new SetSliderCommand(slider, () -> {
        SliderSubsystem.protectionEnabled = true;
        return SliderSubsystem.currentSetPosition;
      }));
      //Reset Gyro D-Pad
      new POVButton(mainControl, 0).whileTrue(new ResetGyroCommand(180).andThen(new ResetDisplacementCommand(new VectorR())));
      //Floor cone detection
      new JoystickButton(mainControl, Button.kX.value)
          .whileTrue(
              new TurnTowardsGamePieceCommand(drive, clawLimelight, LimelightSubsystem.DetectionType.CONE, mainControl));
      //Human player cone detection
      new JoystickButton(mainControl, Button.kB.value)
      .whileTrue(
          new TurnTowardsGamePieceCommand(drive, poleLimelight, LimelightSubsystem.DetectionType.CONE, mainControl));
      //Floor cube detection
      new JoystickButton(mainControl, Button.kA.value)
          .whileTrue(new TurnTowardsGamePieceCommand(drive, clawLimelight, DetectionType.CUBE, mainControl));
      //Pole lineup detection
      new JoystickButton(mainControl, Button.kY.value)
          .whileTrue(new TurnTowardsGamePieceCommand(drive, poleLimelight, DetectionType.RETROREFLECTIVE, mainControl));
    } else {
      //leds.setDefaultCommand(new RunCommand(() -> LEDs.animateLEDs(LEDPattern.STROBE_BLUE), leds));
      drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
      carriage.setDefaultCommand(new RunCommand(() -> carriage.setManual(-1 * mainControl.getRightY()), carriage));
      slider.setDefaultCommand(new RunCommand(() -> slider.setManual(-1 * mainControl.getLeftY()), slider));
      wrist.setDefaultCommand(new RunCommand(() -> wrist.setManual(auxControl.getRightY()), wrist));
      shoulder.setDefaultCommand(new RunCommand(() -> shoulder.setManual(-1 * auxControl.getLeftY() * .35), shoulder));
      intake.setDefaultCommand(new RunCommand(() -> intake.set(0.0), intake));
      
    }
  }

  public void testInit() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}