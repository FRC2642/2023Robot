// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.utils.MathR;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;



public class MoveCarriageCommand extends CommandBase {

  //imports
  private XboxController control;
  private CarriageSubsystem carriage;
  public PIDController carriagePID = new PIDController(.01, 0, 0);
  private boolean extended = false;
  private boolean override = true;

  /** Creates a new CarriageMoveCommand. */
  public MoveCarriageCommand(CarriageSubsystem carriage, XboxController auxControl) {
    this.control = auxControl;
    this.carriage = carriage;
    addRequirements(carriage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    carriage.resetCarriageEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make sure robot wont pull claw into sliders
    /*if (control.getAButtonPressed() || control.getBButtonPressed()) {
      extended = !extended;
      override = false;
    }

    if (extended){
      carriage.move(-0.5);
    }
    else{
      carriage.move(0.5);
    }*/
    
    if (Math.abs(control.getRightY()) > .1){
      carriage.move(control.getRightY()*0.6);
      override = true;
    }
    else if (Math.abs(control.getRightY()) <= 0.1 && override){
      carriage.move(0);
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
