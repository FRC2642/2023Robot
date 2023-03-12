// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IPositionable;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClawWristSubsystem extends SubsystemBase implements IPositionable<ClawWristSubsystem.WristPosition> {
  
  public static double DEGREES_PER_TICK = 180d/22d;
  public static double MAX_DEGREES = 280d;
  public static double MIN_DEGREES = 27d;
  public static double AT_SETPOINT_THRESHOLD = 10d;

  private final CANSparkMax wristMotor = new CANSparkMax(24, MotorType.kBrushed);
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder(Type.kQuadrature, 4);
  
  private final SparkMaxPIDController wristPIDController = wristMotor.getPIDController();
  private WristPosition currentSetPosition = WristPosition.MANUAL;
  
  public enum WristPosition {
    MANUAL(-1),
    HORIZONTAL2(180),
    HORIZONTAL1(360),
    VERTICAL1(270);

    public double angle;

    private WristPosition(double angle) {
      this.angle = angle;
    }
  }
  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    wristEncoder.setPositionConversionFactor(DEGREES_PER_TICK);
    wristMotor.setInverted(true);
    wristPIDController.setP(0.01);
    wristPIDController.setI(0.04);
    wristPIDController.setD(2e-4);
  }

  public double getWristAngle() {
    return wristEncoder.getPosition();
  }

  public void resetWristEncoder() {
    wristEncoder.setPosition(360);
  }

  //Positive = CCW, negative = CW
  public void set(double speed) {
    currentSetPosition = WristPosition.MANUAL;
    wristMotor.set(speed);
  }

  public void set(WristPosition pos) {
    currentSetPosition = pos;
    wristPIDController.setReference(pos.angle, ControlType.kPosition);
  }

  public boolean atSetPosition() {
    return Math.abs(getWristAngle() - currentSetPosition.angle) < AT_SETPOINT_THRESHOLD;
  }

  public WristPosition getSetPosition() {
    return currentSetPosition;
  }
      
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putBoolean("Wrist At Angle", atSetPosition());
  }
}
