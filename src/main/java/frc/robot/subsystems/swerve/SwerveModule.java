// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

/**
 * Thic class represents a single swerve module. It allows for a desired speed
 * and angle to be set. Motor controllers are interfaced with directly from the
 * update() method.
 */
public class SwerveModule {

  // HARDWARE
  public final WPI_TalonFX angleMotor;
  private final WPI_TalonFX driveMotor;
  public final CANCoder orientationEncoder;

  // INFORMATION
  public final SwerveModuleInfo info;
  private final double defensiveAngleDeg;

  public SwerveModule(SwerveModuleInfo info) {
    this.info = info;
    this.angleMotor = new WPI_TalonFX(info.TURN_ID);
    this.driveMotor = new WPI_TalonFX(info.DRIVE_ID);
    this.orientationEncoder = new CANCoder(info.ENCODER_ID);
    this.defensiveAngleDeg = VectorR.fromCartesian(info.X, info.Y).getAngle();
    angleMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    orientationEncoder.setPosition(0);
    driveMotor.setSelectedSensorPosition(0);
  }

  //RESET METHODS
  public void resetDriveEncoder() {
    driveMotor.setSelectedSensorPosition(0);
  }

  // MODULE WHEEL MEASUREMENTS
  public double getWheelSpeed() {
    return driveMotor.getSelectedSensorVelocity() * Constants.FEET_PER_DISPLACEMENT * (100d/1d);
  }

  private double getWheelPosition() {
    return driveMotor.getSelectedSensorPosition() * Constants.FEET_PER_DISPLACEMENT;
  }

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public double getWheelOrientationDegrees() {
    return orientationEncoder.getAbsolutePosition() - info.ABS_ENCODER_VALUE_WHEN_STRAIGHT;
  }

  public VectorR getVelocity() {
    return VectorR.fromPolar(getWheelSpeed(), getWheelOrientationDegrees());
  }

  private double lastWheelPosition = 0;
  private double increment = 0;

  public VectorR getPositionIncrement() {
    return VectorR.fromPolar(increment, getWheelOrientationDegrees());
  }
  
  private void updateIncrementMeasurement() {
    double pos = getWheelPosition();
    increment = pos - lastWheelPosition;
    lastWheelPosition = pos;
  }

  // MODULE SPEEDS CALCULATIONS
  private VectorR desired = new VectorR();
  private boolean reversed = false;

  private void reverse() {
    reversed = !reversed;
  }

  private double desiredSpeed() {
    if (reversed)
      return desired.getTerminalMagnitude();
    else
      return desired.getMagnitude();
  }

  private double desiredAngle() {
    if (reversed)
      return desired.getTerminalAngle();
    else
      return desired.getAngle();
  }

  /*
   * UPDATE OR STOP METHODS MUST BE CALLED PERIODICALLY 
   * speed 0 min - 1 max, turns module drive wheel
   * angle radians follows coordinate plane standards, sets module wheel to angle
   */
  public void update(double speed, double angleDegrees) {

    desired.setFromPolar(speed, angleDegrees);

    if (Math.abs(MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle())) > 90d)
      reverse();

    double speed_power = MathR.limit(desiredSpeed(), -1.2, 1.2);
    double angle_power = MathR
        .limit(Constants.MODULE_ANGLE_KP * MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle()), -1, 1);
   
  
    driveMotor.set(speed_power); 
    angleMotor.set(angle_power);

    updateIncrementMeasurement();
  }

  public void stop() {
    angleMotor.set(0);
    driveMotor.set(0);
    
    
    updateIncrementMeasurement();
  }

  public void stopDefensively() {
    update(0.0000001,  defensiveAngleDeg);
  }
}
