// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;


public class CarriageMoveCommand extends CommandBase {

  //imports
  private XboxController aux;
  private CarriageSubsystem carriage;

  /** Creates a new CarriageMoveCommand. */
  public CarriageMoveCommand(CarriageSubsystem carriage, XboxController aux) {
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
    if (aux.getRightX() != 0){
        carriage.moveCarriage(aux.getRightX());
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
