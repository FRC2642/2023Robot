// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import edu.wpi.first.math.controller.PIDController;


public class MoveCarriageCommand extends CommandBase {

  //imports
  private XboxController aux;
  private CarriageSubsystem carriage;
  public PIDController carriagePID = new PIDController(.01, 0, 0);
  private boolean extended = false;

  /** Creates a new CarriageMoveCommand. */
  public MoveCarriageCommand(CarriageSubsystem carriage, XboxController aux) {
    this.aux = aux;
    addRequirements(carriage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make sure robot wont pull claw into sliders
    if (aux.getXButtonPressed()) {
      extended = !extended;
    }
    

    if (extended){
      carriage.moveCarriage(carriagePID.calculate(CarriageSubsystem.getCarriageEncoder(), 18/*ish*/));
    }
    else {
      carriage.moveCarriage(carriagePID.calculate(CarriageSubsystem.getCarriageEncoder(), 0/*probably*/));
    } 
    

    if (aux.getRightY() != 0){
        carriage.moveCarriage(aux.getRightY());
     }
      else {
        carriage.moveCarriage(0);
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
