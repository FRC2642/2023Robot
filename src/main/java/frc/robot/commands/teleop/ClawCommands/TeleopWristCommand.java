// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  boolean manualControl = true;

  @Override
  public void execute() {
    if (auxControl.getXButton() || auxControl.getYButton()) manualControl = true;
    else if (auxControl.getPOV() != -1) manualControl = false;

    if (manualControl) {
      
    wrist.setSpeedLimit(0.6);
      if (auxControl.getXButton()){
        wrist.set(-1.0);
      }
      else if (auxControl.getYButton()){
        wrist.set(1.0);
      }
      else{
        wrist.set(0.0);
      }
    }
    else {
      
    wrist.setSpeedLimit(0.3);
    
    if (auxControl.getRawButtonPressed(8)) wrist.set(WristPosition.DIAGONAL1);
    else if (auxControl.getRawButtonPressed(7)) wrist.set(WristPosition.DIAGONAL2);

      switch (auxControl.getPOV()) {
        case 0: {
          wrist.set(WristPosition.HORIZONTAL2);
          break;
        }
        case 90: {
          wrist.set(WristPosition.VERTICAL1);
          break;
        }
        case 270: {
          wrist.set(WristPosition.VERTICAL1);
          break;
        }
        case 180: {
          wrist.set(WristPosition.HORIZONTAL1);
          break;
        }
        default: {
          wrist.set();
        }
      }

    }





    /*
    WristPosition setPosition = WristPosition.HORIZONTAL1;
    
    if (auxControl.getPOV() != -1) overridePID = false;
    
    if (!overridePID){
     switch (auxControl.getPOV()) {
      case 0: {
        setPosition = WristPosition.HORIZONTAL1;
        break;
      }
      case 90: {
        setPosition = WristPosition.VERTICAL1;
        break;
      }
      case 270: {
        setPosition = WristPosition.VERTICAL1;
        break;
      }
      case 180: {
        setPosition = WristPosition.HORIZONTAL2;
        break;
      }
    }
    } 

    if (auxControl.getXButton()){
      wrist.set(-0.5);
      overridePID = true;
    }
    else if (auxControl.getYButton()){
      wrist.set(0.5);
      overridePID = true;
    }

    else if (!overridePID) {
      wrist.set(setPosition);
      System.out.println("Wrist angle: " + setPosition.angle + " Wrist pos: " + setPosition.toString());
    }*/

    /*if (wrist.atSetPosition()) wrist.set(0);
    else{*/
    //}
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
