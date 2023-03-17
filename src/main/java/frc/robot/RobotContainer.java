package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.drive.DriveDirectionCommand;
import frc.robot.commands.autonomous.drive.DriveDistanceCommand;
import frc.robot.commands.autonomous.drive.FollowPathCommand;
import frc.robot.commands.autonomous.fullAutos.ASCUBEAutoCommand;
import frc.robot.commands.teleop.ClawCommands.TeleopWristCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.resetters.ResetWristEncoderCommand;
import frc.robot.path.PiratePath;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;

public class RobotContainer {
  private final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
 // private final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);

  private final DriveSubsystem drive = new DriveSubsystem();
 // private final LimelightSubsystem limelight = new LimelightSubsystem();
 // private final ClawGripperSubsystem clawPneumatics = new ClawGripperSubsystem();
 // private final CarriageSubsystem carriage = new CarriageSubsystem();
 // private final ClawIntakeSubsystem intake = new ClawIntakeSubsystem();
 // private final SliderSubsystem slider = new SliderSubsystem();
 // private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
 // private final ClawWristSubsystem wrist = new ClawWristSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  //private final ASCUBEAutoCommand auto_1_ASCUBE;

  public RobotContainer() {
    //auto_1_ASCUBE = new ASCUBEAutoCommand(drive);
    // Default commands
  //  clawPneumatics.setDefaultCommand(new ClawPneumaticCommand(clawPneumatics, mainControl, auxControl));
  //  carriage.setDefaultCommand(new MoveCarriageCommand(carriage, auxControl));
  //  intake.setDefaultCommand(new ClawIntakeCommand(intake, mainControl, auxControl));
  //  slider.setDefaultCommand(new MoveSliderCommand(slider, auxControl));
  //  shoulder.setDefaultCommand(new MoveShoulder(shoulder, auxControl));
  //  wrist.setDefaultCommand(new ClawWristCommand(wrist, auxControl));

    // Auto options
    autoChooser.setDefaultOption("NO AUTO SELECTED", new WaitCommand(5));
    //autoChooser.setDefaultOption(new , getAutonomousCommand());

    // SmartDashboard
    SmartDashboard.putData(autoChooser);
   // SmartDashboard.putData(new ToggleProtectShoulder(shoulder));
    //SmartDashboard.putData(new ResetWristEncoderCommand(wrist, WristPosition.HORIZONTAL1));
    SmartDashboard.putData(new ResetGyroCommand(0.0));
    SmartDashboard.putData(new ResetDisplacementCommand(new VectorR()));

    // Button bindings
    configureButtonBindings();
  }

  public void autonomousInit() {
    drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));

  }


  public void teleopInit() {
    drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));
    

   
    
   /*  clawPneumatics.setDefaultCommand(new RunCommand(() -> {
      if (mainControl.getLeftX() > 0.5) clawPneumatics.set(true);
      else clawPneumatics.set(false);
    }, clawPneumatics)); 

 
    carriage.setDefaultCommand(new RunCommand(() -> {
      carriage.set(mainControl.getLeftX());
    }, carriage)); 


    intake.setDefaultCommand(new RunCommand(() -> {
      intake.set(mainControl.getLeftX());
    }, intake));


    slider.setDefaultCommand(new RunCommand(() -> {
      slider.set(mainControl.getLeftX());
    }, slider));

 
    shoulder.setDefaultCommand(new RunCommand(() -> {
      shoulder.set(mainControl.getLeftX(), false);
    }, shoulder));*/

  //  wrist.setDefaultCommand(new RunCommand(() -> {
     // wrist.set(mainControl.getLeftX());

   // }, wrist));
   //wrist.setDefaultCommand(new TeleopWristCommand(wrist, mainControl));

    /*
    limelight.setDefaultCommand(new RunCommand(() -> {

    }, limelight));*/

    new POVButton(mainControl, 0).whileTrue(new ResetGyroCommand(0.0));
    new POVButton(mainControl, 0).whileTrue(new ResetDisplacementCommand(VectorR.fromCartesian(0, 0)));

  }


  //test mode doesnt work with controllers
  public void testInit() {
  //  System.out.println("TEST INIT");
  //  drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));



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