// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem;
import frc.robot.subsystems.ClawSubsystems.ClawWristSubsystem.WristPosition;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem.CarriagePosition;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem.ShoulderPosition;
import frc.robot.subsystems.MastSubsystems.SliderSubsystem.SliderPosition;

public class SetRobotConfigurationCommand extends ParallelCommandGroup {
  private final RobotConfiguration configuration;

  public SetRobotConfigurationCommand(RobotConfiguration config, ShoulderSubsystem shoulder, SliderSubsystem slider, CarriageSubsystem carriage, ClawWristSubsystem wrist) {
    configuration = config;
   
    addCommands(
      new SetShoulderCommand(shoulder, () -> configuration.shoulderPos),
      new SetCarriageCommand(carriage, () -> configuration.carriagePos),
      new SetSliderCommand(slider, () -> configuration.sliderPos),
      new SetWristCommand(wrist, ()-> configuration.wristPos)
    );

    
  }

  public enum RobotConfiguration {
    PLACE_CONE_HIGH(SliderPosition.EXTENDED, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_CONE_HIGH, WristPosition.HORIZONTAL2),
    PLACE_FIRST_CONE_HIGH_AUTO(SliderPosition.EXTENDED, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_FIRST_CONE_HIGH_AUTO, WristPosition.HORIZONTAL2),
    PLACE_SECOND_CONE_HIGH_AUTO(SliderPosition.EXTENDED, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_SECOND_CONE_HIGH_AUTO, WristPosition.HORIZONTAL2),
    PICKUP_FLOOR(SliderPosition.RETRACTED, CarriagePosition.RETRACTED, ShoulderPosition.PICKUP_GROUND, WristPosition.HORIZONTAL1),
    PLACE_KOCKED_CONE(SliderPosition.EXTENDED, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_CONE_HIGH, WristPosition.HORIZONTAL1),
    TRAVEL_MODE(SliderPosition.RETRACTED, CarriagePosition.RETRACTED, ShoulderPosition.TRAVEL_MODE, WristPosition.HORIZONTAL1),
    PICKUP_HUMAN_PLAYER(SliderPosition.RETRACTED, CarriagePosition.EXTENDED, ShoulderPosition.PICKUP_HUMANPLAYER, WristPosition.HORIZONTAL2),
    PLACE_CONE_MID(SliderPosition.PARTIALLY, CarriagePosition.EXTENDED, ShoulderPosition.PLACE_CONE_HIGH, WristPosition.HORIZONTAL1),
    SHOOT_CUBE(SliderPosition.RETRACTED, CarriagePosition.RETRACTED, ShoulderPosition.SHOOT_CUBE, WristPosition.HORIZONTAL1),
    CHUTE(SliderPosition.RETRACTED, CarriagePosition.CHUTE, ShoulderPosition.CHUTE, WristPosition.HORIZONTAL2);

    public final SliderPosition sliderPos;
    public final CarriagePosition carriagePos;
    public final ShoulderPosition shoulderPos;
    public final WristPosition wristPos;
    private RobotConfiguration( 
      SliderPosition sliderPos,
      CarriagePosition carriagePos,
      ShoulderPosition shoulderPos, WristPosition wristPos) {
      this.sliderPos = sliderPos;
      this.carriagePos = carriagePos;
      this.shoulderPos = shoulderPos;
      this.wristPos = wristPos;
    }

  }
}
