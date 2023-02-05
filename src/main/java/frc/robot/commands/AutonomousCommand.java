// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends CommandBase {
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand() {
    addRequirements(RobotContainer.DriveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  final PIDController PID = new PIDController(.02, 0, 0);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.DriveSub.GyroYawReset();
    RobotContainer.DriveSub.EncoderReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.DriveSub.drive(-.25, RobotContainer.DriveSub.GetYaw());
    double getYaw = RobotContainer.DriveSub.GetYaw();
    double Getticks = RobotContainer.DriveSub.GetLeftTicks();

    double turnspeed = MathUtil.clamp(PID.calculate(getYaw, 0), -1, 1);

    RobotContainer.DriveSub.drive(-.25, turnspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isrightFinished() {
    return DriveSubsystem.GetLeftTicks <= -15;
  }
  
}
