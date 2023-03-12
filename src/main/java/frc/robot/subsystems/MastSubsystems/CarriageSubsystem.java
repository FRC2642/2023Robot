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
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

public class CarriageSubsystem extends SubsystemBase implements IPositionable<CarriageSubsystem.CarriagePosition> {

  public static final double FULL_EXTENSION_PER_TICK = 1d / 520d;

  private final CANSparkMax carriageMotor = new CANSparkMax(Constants.CARRIAGE_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder carriageEncoder = carriageMotor.getEncoder();
  private final SparkMaxLimitSwitch bottomLimitSwitch = carriageMotor.getReverseLimitSwitch(Type.kNormallyOpen);
  private final SparkMaxLimitSwitch topLimitSwitch = carriageMotor.getForwardLimitSwitch(Type.kNormallyOpen);

  private CarriagePosition currentSetPosition = CarriagePosition.RETRACTED;

  public enum CarriagePosition {
    EXTENDED,
    RETRACTED
  }

  public CarriageSubsystem() {
    carriageMotor.setClosedLoopRampRate(0.5);
    carriageEncoder.setPositionConversionFactor(FULL_EXTENSION_PER_TICK);
  }
  //Positive = up
  @Override
  public void set(CarriagePosition pos) {
    carriageMotor.set(pos == CarriagePosition.EXTENDED ? 0.5 : -0.5);
  }

  public void set(double speed){
    carriageMotor.set(speed);
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

}
