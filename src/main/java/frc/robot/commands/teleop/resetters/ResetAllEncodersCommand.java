// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.swerve.SwerveModules;

public class ResetAllEncodersCommand extends CommandBase {
  CarriageSubsystem carriage;
  ShoulderSubsystem shoulder;
  SliderSubsystem slider;
  /** Creates a new ResetTurnEncoderCommand. */
  DriveSubsystem drive;
  public ResetAllEncodersCommand(
    CarriageSubsystem carriage,
    ShoulderSubsystem shoulder,
    SliderSubsystem slider) {
      this.carriage = carriage;
      this.shoulder = shoulder;
      this.slider = slider;
    addRequirements(carriage, shoulder, slider);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    carriage.resetCarriageEncoder();
    slider.resetSliderEncoder();
    shoulder.resetShoulderEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
