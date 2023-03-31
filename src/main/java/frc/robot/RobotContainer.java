package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.autonomous.drive.DriveDirectionCommand;
import frc.robot.commands.autonomous.drive.DriveDistanceCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.fullAutos.ASCUBEAutoCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopWristCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.commands.teleop.resetters.ResetCarriageEncoderCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.commands.teleop.resetters.ResetSliderEncoderCommand;
import frc.robot.subsystems.DriveSubsystem;
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

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class RobotContainer {
  private final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  private final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);

  private final DriveSubsystem drive = new DriveSubsystem();
  private final LimelightSubsystem clawLimelight = new LimelightSubsystem("limelight");
  //private final LimelightSubsystem poleLimelight = new LimelightSubsystem("polelimelight");
  
  private final ClawGripperSubsystem gripper = new ClawGripperSubsystem();
  private final CarriageSubsystem carriage = new CarriageSubsystem();
  private final ClawIntakeSubsystem intake = new ClawIntakeSubsystem();
  private final SliderSubsystem slider = new SliderSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final ClawWristSubsystem wrist = new ClawWristSubsystem();
  //private final LEDSubsystem leds = new LEDSubsystem();
  private final CANdle candle = new CANdle(0);
  private final int ledCount = 68;
  private final Animation rainbowAnimation = new RainbowAnimation(0.7, 0.8, ledCount);
  private final Animation blueAllianceLarsonAnimation = new LarsonAnimation(0, 0, 255, 0, 0.99, ledCount, BounceMode.Back, 7);
  private final Animation redAllianceLarsonAnimation = new LarsonAnimation(255, 0, 0, 0, 0.99, ledCount, BounceMode.Back, 7); 
  //AddressableLED led = new AddressableLED(3);
  //AddressableLEDBuffer buffer = new AddressableLEDBuffer(1);

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
    autoChooser.addOption("Mobility High Cube", new ScoreAndTaxiAuto(slider, gripper, drive, carriage, shoulder, intake, taxiPath));
    /*autoChooser.addOption("[BSCUBE] Barrier Side Cube",
        new BSCONEAutoCommand(drive, clawLimelight, carriage, slider, shoulder, wrist, intake, gripper));
    autoChooser.addOption("[RSCUBE] Rail Side Cube",
        new BSCONEAutoCommand(drive, clawLimelight, carriage, slider, shoulder, wrist, intake, gripper));*/
    autoChooser.addOption("Balance High Cube", new BALANCEAutoCommand(slider, gripper, drive, carriage, intake));//.alongWith(new RunCommand(() -> shoulder.set(ShoulderPosition.STARTING_CONFIG), shoulder)));
    //autoChooser.addOption("[BSCUBE] Barrier Side 2 Cubes", new BSCUBEAutoCommand(drive, clawLimelight, carriage, slider, shoulder, wrist, intake, gripper));
    // SmartDashboard
    SmartDashboard.putData(autoChooser);

    SmartDashboard.putData(new ResetWristEncoderCommand(WristPosition.HORIZONTAL1));
    //SmartDashboard.putData(new ResetGyroCommand(0.0));
   // SmartDashboard.putData(new ResetDisplacementCommand(new VectorR()));
    SmartDashboard.putData(new ResetSliderEncoderCommand(SliderPosition.RETRACTED));
    SmartDashboard.putData(new ResetCarriageEncoderCommand(CarriagePosition.RETRACTED));

    // Button bindings
    
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
   
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
  //  led.setData(buffer);
   // led.start();
   // for (int i = 0; i < buffer.getLength(); i++) {
   //   buffer.setRGB(i, 50, 205, 50);
   // }

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
      new JoystickButton(auxButtonBoard, 1).onTrue(new SetRobotConfigurationCommand(RobotConfiguration.STARTING_CONFIG, shoulder, slider, carriage));
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
    new POVButton(mainControl, 0).whileTrue(new ResetGyroCommand(180));
 //   new JoystickButton(mainControl, Button.kA.value)
//    .whileTrue(new TurnTowardsVisionCommand(drive, clawLimelight, mainControl, LimelightSubsystem.DetectionType.CONE));
//new JoystickButton(mainControl, Button.kB.value).whileTrue(
 //   new TurnTowardsVisionCommand(drive, clawLimelight, mainControl, LimelightSubsystem.DetectionType.FIDUCIAL));
//new JoystickButton(mainControl, Button.kX.value)
   // .whileTrue(new TurnTowardsGamePieceCommand(drive, clawLimelight, DetectionType.CONE, mainControl));
 //new JoystickButton(mainControl, Button.kY.value).whileTrue(
   // new TurnTowardsVisionCommand(drive, clawLimelight, mainControl, LimelightSubsystem.DetectionType.RETROREFLECTIVE));


      //NEW LIMELIGHT CODE
      new JoystickButton(mainControl, Button.kY.value).whileTrue(new TurnTowardsGamePieceCommand(drive, clawLimelight, DetectionType.RETROREFLECTIVE, mainControl));
    }
    else {
      drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
      carriage.setDefaultCommand(new RunCommand(() -> carriage.setManual(-1 * mainControl.getRightY()), carriage));
      slider.setDefaultCommand(new RunCommand(() -> slider.setManual(-1 * mainControl.getLeftY()), slider));
      wrist.setDefaultCommand(new RunCommand(() -> wrist.set(0.0), wrist));
      shoulder.setDefaultCommand(new RunCommand(() -> shoulder.setManual(-1 * auxControl.getLeftY() * .35), shoulder));
      intake.setDefaultCommand(new RunCommand(() -> intake.set(0.0), intake));
    }
  }

  public void testInit() {

  }

  private void configureButtonBindings() {
  //  new POVButton(mainControl, 0).whileTrue(new ResetGyro(drive));
 //   new POVButton(mainControl, 180).whileTrue(new RampCommand(drive, VectorR.fromCartesian(0, 0), true));
  //  new POVButton(mainControl, 270).whileTrue(new ToggleStopDefensivelyCommand(drive));
  //  new JoystickButton(mainControl, Button.kA.value)
 //       .whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.CONE));
  //  new JoystickButton(mainControl, Button.kB.value).whileTrue(
 //       new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.FIDUCIAL));
 //   new JoystickButton(mainControl, Button.kX.value)
 //       .whileTrue(new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.CUBE));
 //   new JoystickButton(mainControl, Button.kY.value).whileTrue(
 //       new TurnTowardsVisionCommand(drive, limelight, mainControl, LimelightSubsystem.DetectionType.RETROREFLECTIVE));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}