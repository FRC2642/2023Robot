// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MastSubsystems.CarriageSubsystem;
import frc.robot.subsystems.MastSubsystems.ShoulderSubsystem;
import frc.robot.subsystems.interfaces.IPositionable;
import frc.robot.utils.MathR;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClawWristSubsystem extends SubsystemBase implements IPositionable<ClawWristSubsystem.WristPosition> {
  
  public static final double DEGREES_PER_TICK = 180d/22d;
  public static final double MAX_DEGREES = 280d;
  public static final double MIN_DEGREES = 27d;
  public static final double AT_SETPOINT_THRESHOLD = 1d;

  private final CANSparkMax wristMotor = new CANSparkMax(24, MotorType.kBrushed);
  private static RelativeEncoder wristEncoder;
  
  private final PIDController wristPIDController = new PIDController/*(0.02, 0.02, 0.0);*/(0.02, 0.008, 0.0);
  private WristPosition currentSetPosition = WristPosition.MANUAL;
  private double speedLimit = 1.0;
  
  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    wristEncoder = wristMotor.getEncoder(Type.kQuadrature, 4);
    wristEncoder.setPositionConversionFactor(DEGREES_PER_TICK);
    wristEncoder.setInverted(true);
    wristMotor.setInverted(false);
    wristPIDController.setTolerance(AT_SETPOINT_THRESHOLD);
  }

  public static double getWristAngle() {
    return wristEncoder.getPosition();
  }

  public static void resetWristEncoder(WristPosition pos) {
    wristEncoder.setPosition(pos.angle);
  }

  //Positive = CCW, negative = CW
  public void set(double speed) {
    currentSetPosition = WristPosition.MANUAL;

    if (ShoulderSubsystem.getShoulderAngle() <= 30 && CarriageSubsystem.getCarriageExtension() <= 0.2){
      wristMotor.set(0.0);
    }
    else{
      wristMotor.set(MathR.limit(speed, -speedLimit, speedLimit));
    }
    
  }

  public void set(WristPosition pos) {
    currentSetPosition = pos;
    if (!atSetPosition()) set(wristPIDController.calculate(getWristAngle(), pos.angle));
    else set(0.0);
    currentSetPosition = pos;
  }
  
  public void set() {
    set(currentSetPosition);
  }

  public boolean atSetPosition() {
    return Math.abs(getWristAngle() - currentSetPosition.angle) < AT_SETPOINT_THRESHOLD;
  }

  public WristPosition getSetPosition() {
    return currentSetPosition;
  }

  @Override
  public void setSpeedLimit(double max) {
    speedLimit = max;
  }

  @Override
  public double getSpeedLimit() {
    return speedLimit;
  }

  @Override
  public void setRampRate(double rampRate) {
    wristMotor.setOpenLoopRampRate(rampRate);
  }

  @Override
  public double getRampRate() {
    return wristMotor.getOpenLoopRampRate();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
   // SmartDashboard.putBoolean("Wrist At Angle", atSetPosition());
   // SmartDashboard.putString("Wrist", currentSetPosition.toString());
  }
  
  public enum WristPosition {
    MANUAL(-1),
    HORIZONTAL2(360),
    HORIZONTAL1(180),
    DIAGONAL1(225),
    DIAGONAL2(315),
    VERTICAL1(270);

    public double angle;

    private WristPosition(double angle) {
      this.angle = angle;
    }
  }
}
