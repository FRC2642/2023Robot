// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;

public class TeleopWristCommand extends CommandBase {

  private final XboxController auxControl;
  private final ClawWristSubsystem wrist;

  public TeleopWristCommand(ClawWristSubsystem wrist, XboxController auxControl) {
    this.auxControl = auxControl;
    this.wrist = wrist;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.setSpeedLimit(1.0);
    wrist.setRampRate(0.0);
  }

  @Override
  public void execute() {
    switch (auxControl.getPOV()) {
      case 0: {
        wrist.set(WristPosition.HORIZONTAL1);
      }
      case 270: {
        wrist.set(WristPosition.VERTICAL1);
      }
      case 180: {
        wrist.set(WristPosition.HORIZONTAL2);
      }
    }

    if (true || wrist.atSetPosition()) wrist.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
