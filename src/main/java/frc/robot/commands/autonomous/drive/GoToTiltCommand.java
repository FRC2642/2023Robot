// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class GoToTiltCommand extends CommandBase {
  /** Creates a new GoToTiltCommand. */
  private final DriveSubsystem drive;
  private final double speed;
  private double startHeading = 0.0;
  private final double angle;
  private final boolean greaterThan;

  private PIDController headingController = new PIDController(0.01, 0, 0);
  private PIDController YController = new PIDController(0.2, 0, 0);

  public GoToTiltCommand(DriveSubsystem drive, double speed, double angle, boolean greaterThan) {
    this.drive = drive;
    this.speed = speed;
    this.angle = angle;
    this.greaterThan = greaterThan;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startHeading = DriveSubsystem.getYawDegrees();
    headingController.setSetpoint(startHeading);
    YController.setSetpoint(DriveSubsystem.getRelativeFieldPosition().getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double angle = Math.PI;
    
    
   
    
    
    // movement = MathR.limit(pid.calculate(DriveSubsystem.getRoll(), 0), -0.3, 0.3) * 0.4;
    
    // System.out.println(movement);
    drive.move(VectorR.fromCartesian(-speed, YController.calculate(DriveSubsystem.getRelativeFieldPosition().getY())), headingController.calculate(DriveSubsystem.getYawDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // System.out.println("ROLL: " + DriveSubsystem.getRoll() + " COND: " + (DriveSubsystem.getRoll() >= -8));
    return greaterThan ? DriveSubsystem.getRoll() >= angle : DriveSubsystem.getRoll() <= angle;
  }
}
