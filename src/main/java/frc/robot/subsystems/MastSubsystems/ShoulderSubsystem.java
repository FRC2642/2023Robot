// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.fasterxml.jackson.annotation.JsonInclude.Include;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.IPositionable;
import frc.robot.utils.MathR;

public class ShoulderSubsystem extends SubsystemBase implements IPositionable<ShoulderSubsystem.ShoulderPosition> {

  public static final double DEGREES_PER_TICK = -1 * 360d / 3.3d;
  public static final double INCLINE_DEGREES = 23d;
  public static final double MAX_DEGREES = 180;//213;
  public static final double MIN_DEGREES = 30;//20;
  public static final double AT_SETPOINT_THRESHOLD = 3d;

  private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR_1, MotorType.kBrushed);
  private final CANSparkMax shoulderMotorFollower = new CANSparkMax(Constants.SHOULDER_MOTOR_2, MotorType.kBrushed);
  private static SparkMaxAnalogSensor absEncoder;

  private final PIDController shoulderPIDController = new PIDController(0.01, 0.0, 0.0);
  private ShoulderPosition currentSetPosition = ShoulderPosition.STARTING_CONFIG;
  private double speedLimit = 0.3;

  public ShoulderSubsystem() {
    absEncoder = shoulderMotor.getAnalog(Mode.kAbsolute);
    absEncoder.setPositionConversionFactor(1.0);
    shoulderPIDController.setTolerance(AT_SETPOINT_THRESHOLD);
    shoulderMotor.setInverted(false);
    // shoulderMotorFollower.setInverted(false);
    shoulderMotorFollower.follow(shoulderMotor);
  }

 // public static double SLOW_DOWN_MAX_ANGLE = 180;
 // public static double SLOW_DOWN_MIN_ANGLE = 50;

  // Negative = up, Positive = down
  public void set(double speed) {
    currentSetPosition = ShoulderPosition.MANUAL;

    /*if ((speed > 0 && getShoulderAngle() > 90) || (speed < 0 && getShoulderAngle() < 90)) {
      speed *= (Math.abs(Math.sin(Math.toRadians(getShoulderAngle() - INCLINE_DEGREES)))+0.1);
    } HIGHLY EXPIERMENTNTIAL !!! :) */

    speed *= (Math.abs(Math.cos(Math.toRadians(getShoulderAngle()))) + 0.1);

    shoulderMotor.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getShoulderAngle() <= MIN_DEGREES,
            getShoulderAngle() >= MAX_DEGREES));
  }

  public void set(ShoulderPosition pos) {
    double speed = shoulderPIDController.calculate(getShoulderAngle(), pos.angle);

    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  public boolean atSetPosition() {
    return shoulderPIDController.atSetpoint();
  }

  public ShoulderPosition getSetPosition() {
    return currentSetPosition;
  }

  public static double getShoulderAngle() {
    return absEncoder.getPosition() * DEGREES_PER_TICK + 270;
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
    shoulderMotor.setOpenLoopRampRate(rampRate);
  }

  @Override
  public double getRampRate() {
    return shoulderMotor.getOpenLoopRampRate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
    SmartDashboard.putString("Shoulder", currentSetPosition.toString());
  }

  public enum ShoulderPosition {
    MANUAL(-1),
    STARTING_CONFIG(56),
    PICKUP_GROUND(MAX_DEGREES),
    PICKUP_HUMANPLAYER(MIN_DEGREES),
    PLACE_CUBE_MID(180),
    PLACE_CUBE_HIGH(150),
    PLACE_CONE_MID(170),
    PLACE_CONE_HIGH(MIN_DEGREES);

    public double angle;

    private ShoulderPosition(double angle) {
      this.angle = angle;
    }
  }
}
