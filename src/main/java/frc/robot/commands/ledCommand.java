// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.XboxController;

public class ledCommand extends CommandBase {
  private final LEDs ledS;
  private final XboxController main;
  public ledCommand(LEDs ledS, XboxController main) {
    this.ledS = ledS;
    this.main = main;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mainControl.getAButton() == true){
      LEDs.setLEDCOLOR(128,0,128);
    }
    else if(mainControl.getXButton() == true || mainControl.getBButton() == true){
      LEDs.setLEDCOLOR(255, 255, 0);
    }
    else if (DriverStation.getAlliance() == Alliance.Blue){
      LEDs.setLEDCOLOR(0, 0, 255);
    }
    else if (DriverStation.getAlliance() == Alliance.Red){
      LEDs.setLEDCOLOR(255, 0, 0);
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
