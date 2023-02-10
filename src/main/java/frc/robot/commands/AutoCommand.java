// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoCommand extends CommandBase {
  /** Creates a new AutonomousCommand. */
  public AutoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.DriveSub);
  
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

    
    RobotContainer.DriveSub.arcadeDrive(-0.25, RobotContainer.DriveSub.GetYaw());
    double GetYaw = RobotContainer.DriveSub.GetYaw();
    double GetTicks = RobotContainer.DriveSub.GetLeftTicks();

    double turnspeed = MathUtil.clamp(PID.calculate(GetYaw,0), -1, 1);

    
    RobotContainer.DriveSub.arcadeDrive(-.25, turnspeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean IsLeftFinished() {
    return RobotContainer.DriveSub.GetLeftTicks() <= -15;
  }
}
