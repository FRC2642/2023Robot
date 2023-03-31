// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.XboxController;

public class ledCommand extends CommandBase {
  private final LEDs leds;
  private final XboxController main;
  private final Joystick buttonBoard;
  
  final Animation flowAnimation = new ColorFlowAnimation(50, 205, 50, 255, 1, Constants.LED_LENGTH, Direction.Forward);

  public ledCommand(LEDs leds, XboxController main, Joystick buttonBoard) {
    this.leds = leds;
    this.main = main;
    this.buttonBoard = buttonBoard;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (buttonBoard.getRawButton(10)){
      LEDs.candle.animate(flowAnimation);
    }
    else if (main.getAButton()){
      LEDs.setLEDCOLOR(100, 0, 200);
    }
    else if (main.getBButton() || main.getXButton()){
      LEDs.setLEDCOLOR(0, 255, 255);
    }
    else{
      if (DriverStation.getAlliance() == Alliance.Blue){
        LEDs.setLEDCOLOR(0, 0, 255);
      }
      else{
        LEDs.setLEDCOLOR(255, 0, 0);
      }
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
