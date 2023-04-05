// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;

public class SetRobotConfigurationCommand extends ParallelCommandGroup {
  private final RobotConfiguration configuration;

  public SetRobotConfigurationCommand(RobotConfiguration config, ShoulderSubsystem shoulder, SliderSubsystem slider, CarriageSubsystem carriage) {
    configuration = config;
   
    addCommands(
      new SetShoulderCommand(shoulder, () -> configuration.shoulderPos),
      new SetCarriageCommand(carriage, () -> configuration.carriagePos),
      new SetSliderCommand(slider, () -> configuration.sliderPos)
    );

    
  }

  public enum RobotConfiguration {
    PLACE_CONE_HIGH(SliderPosition.EXTENDED, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_CONE_HIGH),
    PICKUP_FLOOR(SliderPosition.RETRACTED, CarriagePosition.RETRACTED, ShoulderPosition.PICKUP_GROUND),
    TRAVEL_MODE(SliderPosition.RETRACTED, CarriagePosition.RETRACTED, ShoulderPosition.TRAVEL_MODE),
    PICKUP_HUMAN_PLAYER(SliderPosition.RETRACTED, CarriagePosition.EXTENDED, ShoulderPosition.PICKUP_HUMANPLAYER),
    PLACE_CONE_MID(SliderPosition.PARTIALLY, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_CONE_HIGH);

    public final SliderPosition sliderPos;
    public final CarriagePosition carriagePos;
    public final ShoulderPosition shoulderPos;
    private RobotConfiguration( 
      SliderPosition sliderPos,
      CarriagePosition carriagePos,
      ShoulderPosition shoulderPos) {
      this.sliderPos = sliderPos;
      this.carriagePos = carriagePos;
      this.shoulderPos = shoulderPos;
    }

  }
}
