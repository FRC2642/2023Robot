package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
import frc.robot.commands.autonomous.fullAutos.BSCONEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.ScoreAndTaxiAuto;
import frc.robot.commands.autonomous.positionable.SetCarriageCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.autonomous.positionable.SetShoulderCommand;
import frc.robot.commands.autonomous.positionable.SetSliderCommand;
import frc.robot.commands.autonomous.positionable.SetWristCommand;
import frc.robot.commands.autonomous.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.autonomous.fullAutos.BALANCEAutoCommand;
import frc.robot.commands.autonomous.fullAutos.BSCUBEAutoCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopGripperCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopIntakeCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopWristCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsGamePieceCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsVisionCommand;
import frc.robot.commands.teleop.MastCommands.TeleopCarriageCommand;
import frc.robot.commands.teleop.MastCommands.TeleopShoulderCommand;
import frc.robot.commands.teleop.MastCommands.TeleopSliderCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
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
import com.mindsensors.*;

public class RobotContainer {
  private final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  private final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  private final DriveSubsystem drive = new DriveSubsystem();
  private final LimelightSubsystem clawLimelight = new LimelightSubsystem("limelight");
  private final LimelightSubsystem poleLimelight = new LimelightSubsystem("polelimelight");
  
  private final ClawGripperSubsystem gripper = new ClawGripperSubsystem();
  private final CarriageSubsystem carriage = new CarriageSubsystem();
  private final ClawIntakeSubsystem intake = new ClawIntakeSubsystem();
  private final SliderSubsystem slider = new SliderSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final ClawWristSubsystem wrist = new ClawWristSubsystem();
  //private final LEDSubsystem leds = new LEDSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final PiratePath taxiPath = new PiratePath();

  public static boolean DEBUG = false;

  //public static final AddressableLED leds = new AddressableLED(Constants.LED_PORT);
  //private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED_LENGTH);

  // private final ASCUBEAutoCommand auto_1_ASCUBE;

  public RobotContainer() {

    /*leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();*/
    taxiPath.add(new PiratePoint(0, 0, 180, 0, false));
    taxiPath.add(new PiratePoint(14.5, 0, 180, 8, false));
    taxiPath.fillWithSubPointsEasing(0.01, Functions.easeInOutCubic);

    SmartDashboard.putNumber("DEBUG MODE", 0);

    // auto_1_ASCUBE = new ASCUBEAutoCommand(drive);
    // Default commands

    // Auto options
    autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(5));
    autoChooser.addOption("drop and move", new ScoreAndTaxiAuto(slider, gripper, drive, carriage, shoulder, intake, taxiPath));
    autoChooser.addOption("[BSCUBE] Barrier Side Cube",
        new BSCONEAutoCommand(drive, clawLimelight, carriage, slider, shoulder, wrist, intake, gripper));
    autoChooser.addOption("[RSCUBE] Rail Side Cube",
        new BSCONEAutoCommand(drive, clawLimelight, carriage, slider, shoulder, wrist, intake, gripper));
    autoChooser.addOption("Balance", new BALANCEAutoCommand(slider, gripper, drive, carriage, shoulder));
    autoChooser.addOption("[BSCUBE] Barrier Side 2 Cubes", new BSCUBEAutoCommand(drive, clawLimelight, carriage, slider, shoulder, wrist, intake, gripper));
    // SmartDashboard
    SmartDashboard.putData(autoChooser);

    SmartDashboard.putData(new ResetWristEncoderCommand(WristPosition.HORIZONTAL1));
    SmartDashboard.putData(new ResetGyroCommand(0.0));
    SmartDashboard.putData(new ResetDisplacementCommand(new VectorR()));
    SmartDashboard.putData(new ResetSliderEncoderCommand(SliderPosition.RETRACTED));
    SmartDashboard.putData(new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED));

    // mindsensors led class
    //CANLight lights = new CANLight(Constants.LED_PORT);
    
    //leds.blink(1);
    // Button bindings
  }

  public void autonomousInit() {
    drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
    carriage.setDefaultCommand(new RunCommand(() -> carriage.set(0.0), carriage));
    slider.setDefaultCommand(new RunCommand(() -> slider.set(0.0), slider));
    wrist.setDefaultCommand(new RunCommand(() -> wrist.set(0.0), wrist));
    shoulder.setDefaultCommand(new RunCommand(() -> shoulder.set(0.0), shoulder));
    intake.setDefaultCommand(new RunCommand(() -> intake.set(0.0), intake));
  }

  public void teleopInit() {

   // leds.blink(2);
    /*for (int i = 0; i < buffer.getLength(); i++){
      buffer.setRGB(i, 120, 190, 33);
    }
    leds.setData(buffer);*/
    CommandScheduler.getInstance().cancelAll();
    SmartDashboard.putData("Starting Config", new SetShoulderCommand(shoulder, () -> ShoulderPosition.STARTING_CONFIG));
    // SmartDashboard.putData(new SetCarriageCommand(carriage,
    // CarriagePosition.EXTENDED));
    // SmartDashboard.putData(new SetWristCommand(wrist,
    // WristPosition.HORIZONTAL1));

    if (!DEBUG) {  
      new JoystickButton(auxControl, 1).onTrue(
      new SetRobotConfigurationCommand(RobotConfiguration.PLACE_CONE_HIGH, shoulder, slider, carriage).withTimeout(5));
     // .raceWith(new SetWristCommand(wrist, () -> WristPosition.HORIZONTAL1)));

      new JoystickButton(auxControl, 2).onTrue(
      new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_FLOOR, shoulder, slider, carriage).withTimeout(5));
      
    //  .raceWith(new SetWristCommand(wrist, () -> WristPosition.HORIZONTAL2)));

      drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));
      gripper.setDefaultCommand(new TeleopGripperCommand(gripper, auxControl));
      carriage.setDefaultCommand(new TeleopCarriageCommand(carriage, auxControl));
      intake.setDefaultCommand(new TeleopIntakeCommand(intake, auxControl));
      slider.setDefaultCommand(new TeleopSliderCommand(slider, auxControl));   
      shoulder.setDefaultCommand(new TeleopShoulderCommand(shoulder, auxControl));
      wrist.setDefaultCommand(new TeleopWristCommand(wrist, auxControl));
      
      //BUTTONS
      new JoystickButton(auxButtonBoard, 8).onTrue(new SetRobotConfigurationCommand(RobotConfiguration.PICKUP_HUMAN_PLAYER, shoulder, slider, carriage));
      new JoystickButton(auxButtonBoard, 12).onTrue(new SetSliderCommand(slider, () -> {
        SliderSubsystem.protectionEnabled = false;
        return SliderPosition.EXTENDED;
      }));
      new JoystickButton(auxButtonBoard, 11).onTrue(new SetSliderCommand(slider, () -> {
        SliderSubsystem.protectionEnabled = false;
        return SliderPosition.RETRACTED;
      }));
      new JoystickButton(auxButtonBoard, 10).onTrue(new SetSliderCommand(slider, () -> {
        SliderSubsystem.protectionEnabled = true;
        return SliderSubsystem.currentSetPosition;
      }));
    new POVButton(mainControl, 0).whileTrue(new ResetGyroCommand(0));
    new JoystickButton(mainControl, Button.kA.value)
    .whileTrue(new TurnTowardsVisionCommand(drive, clawLimelight, mainControl, LimelightSubsystem.DetectionType.CONE));
new JoystickButton(mainControl, Button.kB.value).whileTrue(
    new TurnTowardsVisionCommand(drive, clawLimelight, mainControl, LimelightSubsystem.DetectionType.FIDUCIAL));
new JoystickButton(mainControl, Button.kX.value)
    .whileTrue(new TurnTowardsGamePieceCommand(drive, clawLimelight, DetectionType.CONE, mainControl));
 //new JoystickButton(mainControl, Button.kY.value).whileTrue(
   // new TurnTowardsVisionCommand(drive, clawLimelight, mainControl, LimelightSubsystem.DetectionType.RETROREFLECTIVE));


      //NEW LIMELIGHT CODE
      //new JoystickButton(mainControl, Button.kY.value).whileTrue(new TurnTowardsGamePieceCommand(drive, poleLimelight, DetectionType.RETROREFLECTIVE));
    }
    else {
      drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
      carriage.setDefaultCommand(new RunCommand(() -> carriage.setManual(-1 * mainControl.getRightY()), carriage));
      slider.setDefaultCommand(new RunCommand(() -> slider.setManual(-1 * mainControl.getLeftY()), slider));
      wrist.setDefaultCommand(new RunCommand(() -> wrist.set(0.0), wrist));
      shoulder.setDefaultCommand(new RunCommand(() -> shoulder.setManual(-1 * auxControl.getLeftY()), shoulder));
      intake.setDefaultCommand(new RunCommand(() -> intake.set(0.0), intake));
    }
  }

  public void testInit() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}