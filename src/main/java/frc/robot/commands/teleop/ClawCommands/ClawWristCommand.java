// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;


public class ClawWristCommand extends CommandBase {
  private XboxController aux;
  private ClawWristSubsystem wrist;
  //double position;
  //double speed;
  /** Creates a new ClawWristDirection. */
  public ClawWristCommand(XboxController aux, ClawWristSubsystem wrist, double position, double speed) {
    this.aux = aux;
    this.wrist = wrist;
    //this.position = position;
    //this.speed = speed;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.resetEncoder();
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (aux.getXButtonPressed() && !wrist.clawInRobot()){
      wrist.flipClaw();
    }
    /*if (limelight.getDetectionType() == "CONE"){
      if (limelight.getWidth() - limelight.getHeight() <= 1){
        wrist.moveWrist(-1);
      }
    }*/

    // If limit switch is hit, prevents wrist from moving wrist in the same direction further
    if (wrist.getLimitSwitchState() == true) {
      // prevents wrist from twisting inside of robot
      if (CarriageSubsystem.isCarriageFullyRetracted() && SliderSubsystem.isSliderBack() && ShoulderSubsystem.isShoulderBack()){
        if (ClawWristSubsystem.getEncoderTicks() > 0 && aux.getLeftX() < 0) {
          wrist.setWrist(aux.getLeftX());
        }else if (ClawWristSubsystem.getEncoderTicks() < 0 && aux.getLeftX() > 0) {
          wrist.setWrist(aux.getLeftX());
        }else {
          wrist.stopWrist();
        }
      }
    } else if (!wrist.clawInRobot()) {
      wrist.setWrist(aux.getLeftX());
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
