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
  DriveSubsystem drive;
  double movement = 0;
  PIDController pid = new PIDController(0.1, 0, 0);
  public GoToTiltCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = 0;
    
    if (DriveSubsystem.getRoll() > 0){
      angle = 0;
    }
    else if (DriveSubsystem.getRoll() < 0){
      angle = Math.PI;
    }
    
   
    if (angle == 0){
      movement = MathR.limit(pid.calculate(DriveSubsystem.getRoll(), 2), -0.2, 0.2);
    }
    else if (angle == Math.PI){
      movement = MathR.limit(pid.calculate(DriveSubsystem.getRoll(), -2), -0.2, 0.2);
    }
    System.out.println(movement);
    drive.move(VectorR.fromPolar(movement, angle), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return movement <= 0.1;
  }
}
