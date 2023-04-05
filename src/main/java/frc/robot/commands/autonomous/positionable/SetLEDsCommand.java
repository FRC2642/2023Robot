// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDPattern;

public class SetLEDsCommand extends CommandBase {
  /** Creates a new SetLEDsCommand. */
  private LEDs leds;
  private Supplier<LEDPattern> pattern;
  public SetLEDsCommand(LEDs leds, Supplier<LEDPattern> pattern) {
    this.leds = leds;
    this.pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.animateLEDs(pattern.get());
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
