// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ClawSubsystems.ClawIntakeSubsystem;
import frc.robot.subsystems.LEDs.LEDPattern;

public class SetLEDsCommand extends CommandBase {
  /** Creates a new SetLEDsCommand. */
  private XboxController mainControl;
  private XboxController auxControl;
  public SetLEDsCommand(LEDs leds, XboxController mainControl, XboxController auxControl) {
    this.mainControl = mainControl;
    this.auxControl = auxControl;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mainControl.getRightStickButton()){
      LEDs.animateLEDs(LEDPattern.STROBE_YELLOW);
    }
    else if (mainControl.getLeftStickButton()){
      LEDs.animateLEDs(LEDPattern.STROBE_PURPLE);
    }
    else if (auxControl.getRightStickButton()){
      LEDs.animateLEDs(LEDPattern.RAINBOW);
    }
    else if (Math.abs(DriveSubsystem.getRollDegrees()) > 20 || Math.abs(DriveSubsystem.getPitchDegrees()) > 20){
      LEDs.animateLEDs(LEDPattern.FLASHING_RED);
    }
    else{
      LEDs.animateLEDs(LEDPattern.OFF);
    }
    
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
