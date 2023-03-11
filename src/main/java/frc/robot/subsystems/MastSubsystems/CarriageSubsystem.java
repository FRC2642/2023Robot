// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.IPositionable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CarriageSubsystem extends SubsystemBase implements IPositionable<CarriageSubsystem.CarriagePosition> {

  public static final double FULL_EXTENSION_PER_TICK = 1d / 520d;

  private final CANSparkMax carriage = new CANSparkMax(Constants.CARRIAGE_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder carriageEncoder;

  private CarriagePosition currentSetPosition = CarriagePosition.RETRACTED;

  public enum CarriagePosition {
    EXTENDED,
    RETRACTED
  }

  public CarriageSubsystem() {
    carriageEncoder = carriage.getEncoder();
    carriageEncoder.setPositionConversionFactor(FULL_EXTENSION_PER_TICK);
  }

  public void set(double speed){
    carriage.set(0);
  }

  public double getCarriageExtension() {
    return carriageEncoder.getPosition();
  }

  

  //reset the carriage position
  public void resetCarriageEncoder() {
    currentSetPosition = CarriagePosition.RETRACTED;
    carriageEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Carriage Extension",carriageEncoder.getPosition());
  }

  @Override
  public boolean atSetPosition() {
    return true;
  }

  @Override
  public CarriagePosition getSetPosition() {
    return currentSetPosition;
  }

  @Override
  public void set(CarriagePosition pos) {
    carriage.stopMotor();
    
  }
}
