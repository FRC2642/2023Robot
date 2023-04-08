// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DivertToGamePieceCommand extends FollowPathCommand {
  /** Creates a new DivertToGamePieceCommand. */
  DriveSubsystem drive;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType object;
  PiratePath path;
  double visionSpeed;
  double timeAfterStartToDivert;
  Timer visionTimer = new Timer();
  ClawIntakeSubsystem intake;
  double intakeSpeed;
  ClawGripperSubsystem gripper;

  public DivertToGamePieceCommand(DriveSubsystem drive, LimelightSubsystem limelight, LimelightSubsystem.DetectionType object, PiratePath path, boolean recenterDisplacementToFirstPoint, double additionalLookaheadTime, double visionSpeed, double timeAfterStartToDivert, ClawIntakeSubsystem intake, double intakeSpeed, ClawGripperSubsystem gripper) {
    super(drive, path, recenterDisplacementToFirstPoint, additionalLookaheadTime);
    this.drive = drive;
    this.limelight = limelight;
    this.object = object;
    this.path = path;
    this.visionSpeed = visionSpeed;
    this.timeAfterStartToDivert = timeAfterStartToDivert;
    this.intake = intake;
    this.gripper = gripper;
    this.intakeSpeed = intakeSpeed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    limelight.setDetectionType(object);
    visionTimer.reset();
    visionTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (visionTimer.get() > timeAfterStartToDivert && limelight.isDetection && limelight.confidence() > 1){
      gripper.set(object == LimelightSubsystem.DetectionType.CUBE);
      intake.set(intakeSpeed);
      drive.move(VectorR.fromPolar(visionSpeed, DriveSubsystem.getYawDegrees() + limelight.x + 180), MathR.limit(limelight.x * -1 * (1d/45d), -0.25, 0.25));
    }
    else{
      super.execute();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClawIntakeSubsystem.isObjectInClaw();
  }
}
