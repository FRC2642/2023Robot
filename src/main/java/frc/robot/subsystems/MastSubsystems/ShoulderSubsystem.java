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

  public static final double DEGREES_PER_TICK = 360d / 3.3d;
  public static final double INCLINE_DEGREES = 27d;
  public static final double MAX_DEGREES = 90 + INCLINE_DEGREES;
  public static final double MIN_DEGREES = -INCLINE_DEGREES;
  public static final double OFFSET_DEGREES = 0;

  private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushed);
  private final PIDController shoulderPIDController = new PIDController(0.2, 0.0, 0.0);
  private final SparkMaxAnalogSensor absEncoder;

  ShoulderPosition currentSetPosition = ShoulderPosition.STARTING_CONFIG;
  // private static SparkMaxLimitSwitch frontShoulderLimitSwitch;
  // private static SparkMaxLimitSwitch rearShoulderLimitSwitch;

  public enum ShoulderPosition {
    MANUAL(-1),
    STARTING_CONFIG(70),
    PICKUP_GROUND(MAX_DEGREES),
    PICKUP_HUMANPLAYER(MIN_DEGREES),
    PLACE_CUBE1(180),
    PLACE_CUBE2(150),
    PLACE_CONE1(170),
    PLACE_CONE1_OFFSIDE(MIN_DEGREES),
    PLACE_CONE2_OFFSIDE(MIN_DEGREES);

    public double angle;

    private ShoulderPosition(double angle) {
      this.angle = angle;
    }
  }


  public ShoulderSubsystem() {
    absEncoder = shoulderMotor.getAnalog(Mode.kAbsolute);
    absEncoder.setPositionConversionFactor(DEGREES_PER_TICK);
  }

  //Negative = down, Positive = up
  public void set(double speed, boolean softLimit) {
    currentSetPosition = ShoulderPosition.MANUAL;
    if (softLimit)
      shoulderMotor.set(
          MathR.limitWhenReached(speed, -1, 1, getShoulderAngle() <= MIN_DEGREES, getShoulderAngle() >= MAX_DEGREES));
    else
      shoulderMotor.set(speed);
  }

  public void set(ShoulderPosition pos) {
    currentSetPosition = pos;
    double speed = MathR.limit(MathR.getDistanceToAngleRadians(getShoulderAngle(), pos.angle), -1, 1);
    
    shoulderMotor.set(MathR.limitWhenReached(speed, -1, 1, getShoulderAngle() <= MIN_DEGREES, getShoulderAngle() >= MAX_DEGREES));
  }

  public boolean atSetPosition() {
    return shoulderPIDController.atSetpoint();
  }
  public ShoulderPosition getSetPosition() {
    return currentSetPosition;
  }

  public double getShoulderAngle() {
    return absEncoder.getPosition() + OFFSET_DEGREES;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());

  }
}
