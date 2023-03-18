// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.MastCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import edu.wpi.first.math.controller.PIDController;

public class TeleopCarriageCommand extends CommandBase {

  private final XboxController auxControl;
  private final CarriageSubsystem carriage;
  private boolean extended = false;

  public TeleopCarriageCommand(CarriageSubsystem carriage, XboxController auxControl) {
    this.auxControl = auxControl;
    this.carriage = carriage;
    addRequirements(carriage);
  }
  
  @Override
  public void initialize() {
    carriage.setSpeedLimit(0.5);
    carriage.setRampRate(0.25);
  }

  @Override
  public void execute() {
    //make sure robot wont pull claw into sliders
  /*   if (auxControl.getXButtonPressed()) {
      extended = !extended;
    }
    carriage.set(extended ? CarriagePosition.EXTENDED : CarriagePosition.MANUAL); */
    if (Math.abs(auxControl.getRightY()) > 0.1) {
      carriage.set(-1 * auxControl.getRightY());
    }
    else carriage.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
