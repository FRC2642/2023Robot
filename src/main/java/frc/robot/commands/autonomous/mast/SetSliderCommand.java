// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.mast;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;

public class SetSliderCommand extends CommandBase {
  /** Creates a new SetSliderCommand. */
  SliderSubsystem sliders;
  boolean extend;
  public SetSliderCommand(SliderSubsystem sliders, boolean extend) {
    this.sliders = sliders;
    this.extend = extend;
    addRequirements(sliders);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sliders.move(extend);
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
