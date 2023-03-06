// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class JoystickOrientedDriveCommand extends CommandBase {

  double maxSpeed = 0.25;

  final DriveSubsystem drive;

  // CONTROLLER DATA
  final XboxController control;
  final VectorR leftJoystick = new VectorR();
  final VectorR rightJoystick = new VectorR();

  public JoystickOrientedDriveCommand(DriveSubsystem drive, XboxController control) {
    this.drive = drive;
    this.control = control;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveSubsystem.resetGyro(0.0);
  }

  // JOYSTICK ORIENTED CONTROLLER
  //final PIDController pid = new PIDController(0.95, 0.0, 0.0);
  final double TURN_KP = 0.8;
  private boolean isLocked = false;
  private double lockedHeading = 0;

  @Override
  public void execute() {
    if (DriveSubsystem.getPitch() <= 5 || DriveSubsystem.getRoll() <= 5){
    
      maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis());

      //MAX_SPEED = (control.getRightTriggerAxis())/(2.0)+0.5;
      

      leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
      leftJoystick.rotate(Math.toRadians(90));
      rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
      rightJoystick.rotate(Math.toRadians(-90));

      double yaw = Math.toRadians(DriveSubsystem.getYawDegrees());

      if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() < 0.2) {
        drive.stop();
        isLocked = false;
        return;
      }

      if (leftJoystick.getMagnitude() > 0.1 && rightJoystick.getMagnitude() < 0.2) {
        if (!isLocked) {
          lockedHeading = yaw;
          isLocked = true;
        }
      } 
      else if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() > 0.2) {
        leftJoystick.setFromCartesian(0.0, 0.0);
      }
      else isLocked = false;

      double angleToFace = isLocked ? lockedHeading : rightJoystick.getAngle();

      double turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
          .limit(TURN_KP * MathR.getDistanceToAngleRadians(yaw, angleToFace), -1, 1);

      leftJoystick.mult(maxSpeed);
      drive.move(leftJoystick, turnPower * maxSpeed);
    }
    else{
      if (DriveSubsystem.getPitch() <= -5){
        drive.move(VectorR.fromPolar(0.6, Math.toRadians(DriveSubsystem.getYawDegrees())), 0);
      }
      else if (DriveSubsystem.getPitch() >= 5){
        drive.move(VectorR.fromPolar(0.6, Math.PI + Math.toRadians(DriveSubsystem.getYawDegrees())), 0);
      }
      else if (DriveSubsystem.getRoll() <= -5){
        drive.move(VectorR.fromPolar(0.6, (Math.PI/2) + Math.toRadians(DriveSubsystem.getYawDegrees())), 0);
      }
      else if (DriveSubsystem.getRoll() >= 5){
        drive.move(VectorR.fromPolar(0.6, (3*Math.PI/2) + Math.toRadians(DriveSubsystem.getYawDegrees())), 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
