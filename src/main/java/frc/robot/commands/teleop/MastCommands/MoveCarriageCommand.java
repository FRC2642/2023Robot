// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.utils.MathR;
import edu.wpi.first.math.controller.PIDController;



public class MoveCarriageCommand extends CommandBase {

  //imports
  private XboxController control;
  private CarriageSubsystem carriage;
  public PIDController carriagePID = new PIDController(.01, 0, 0);
  private boolean extended = false;
  private double dampen;

  /** Creates a new CarriageMoveCommand. */
  public MoveCarriageCommand(CarriageSubsystem carriage, XboxController auxControl) {
    this.control = auxControl;
    this.carriage = carriage;
    addRequirements(carriage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make sure robot wont pull claw into sliders
    if (control.getXButtonPressed()) {
      extended = !extended;
    }
    

    /*if (extended){
      carriage.move(MathR.limit(carriagePID.calculate(CarriageSubsystem.getCarriageEncoder(), 18), -1, 1/*ish));
    }
    else {
      carriage.move(MathR.limit(carriagePID.calculate(CarriageSubsystem.getCarriageEncoder(), 0/*probably), -1, 1));
    } */
    

    if (Math.abs(control.getRightY()) > .1){
        //carriage.move(control.getRightY());

        //Slows carriage down if approaching the back of the robot
        /*if (control.getRightY() < 0){
          dampen = MathR.limit(carriagePID.calculate(CarriageSubsystem.getCarriageEncoder(), 0), -1, 1);
          carriage.move(control.getRightY() * dampen);
        }

        //Slows carriage down if approaching the front of the robot
        else{
          dampen = MathR.limit(carriagePID.calculate(CarriageSubsystem.getCarriageEncoder(), 18), -1, 1);
          carriage.move(control.getRightY() * dampen);
        }*/
        carriage.set(control.getRightY()*0.6);
     }
    else {
        carriage.set(0);
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
