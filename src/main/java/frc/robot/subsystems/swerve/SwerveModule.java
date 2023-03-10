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
  public final WPI_TalonFX driveMotor;
  public final CANCoder absEncoder;
  public final CANCoder turnEncoder;

  // INFORMATION
  public final SwerveModuleInfo info;
  private final double absMaxValue;
  private final double absStraightValue;
  public final VectorR position;
  private double totalDegreesTurned;

  public SwerveModule(SwerveModuleInfo info) {
    this.info = info;
    this.angleMotor = new WPI_TalonFX(info.TURN_ID);
    this.driveMotor = new WPI_TalonFX(info.DRIVE_ID);
    this.absMaxValue = info.ABS_ENCODER_MAX_VALUE;
    this.absStraightValue = info.ABS_ENCODER_VALUE_WHEN_STRAIGHT;
    this.position = VectorR.fromCartesian(info.X, info.Y);
    this.absEncoder = new CANCoder(info.ENCODER_ID);
    this.turnEncoder = new CANCoder(info.ENCODER_ID);
    angleMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Brake);

    //angleMotor.configClosedloopRamp(0);
    //driveMotor.configClosedloopRamp(0);
    turnEncoder.setPosition(0);
    driveMotor.setSelectedSensorPosition(0);

    
  }

  //ENOCDER METHODS
  public double getEncoderValue(){
    return driveMotor.getSelectedSensorPosition();
  }

  public void setEncoderValue(double pos){
    driveMotor.setSelectedSensorPosition(pos);
  }

  public double getTurnEncoderValue(){
    return absEncoder.getAbsolutePosition();
  }

  public double getRelativeTurnEncoderValue(){
    return turnEncoder.getPosition();
  }

  //RESET METHODS
  public void resetDriveEncoder() {
    setEncoderValue(0.0);
  }

  // MODULE WHEEL MEASUREMENTS
  public double getWheelSpeed() {
    return driveMotor.getSelectedSensorVelocity() * Constants.FEET_PER_DISPLACEMENT * (1d/60d);
  }

  private double getWheelPosition() {
    return getEncoderValue() * Constants.FEET_PER_DISPLACEMENT;
  }

  public double getWheelPositionWithoutDrift(){
    return getWheelPosition() - (Constants.DRIFT_PER_DEGREE * totalDegreesTurned);
  }

  

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public double getWheelHeadingRadians() {
    return (((getTurnEncoderValue() - absStraightValue) / absMaxValue) * Math.PI * 2.0);
  }

  public VectorR getVelocity() {
    return VectorR.fromPolar(getWheelSpeed(), getWheelHeadingRadians());
  }

  private double lastWheelPosition = 0;
  private double increment = 0;

  public VectorR getPositionIncrement() {
    return VectorR.fromPolar(increment, getWheelHeadingRadians());
  }
  private void updateIncrementMeasurement() {
    double pos = getWheelPositionWithoutDrift();
    
    increment = pos - lastWheelPosition;
    lastWheelPosition = pos;
  }

  public void updateTotalDegreesTurned(){
    totalDegreesTurned = turnEncoder.getPosition();
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
  public void update(double speed, double anglerad) {

    desired.setFromPolar(speed, anglerad);

    if (Math.abs(MathR.getDistanceToAngleRadians(getWheelHeadingRadians(), desiredAngle())) > Math.toRadians(90))
      reverse();

    double speed_power = MathR.limit(desiredSpeed(), -1, 1);
    double angle_power = 1 * MathR
        .limit(Constants.MODULE_ANGLE_KP * MathR.getDistanceToAngleRadians(getWheelHeadingRadians(), desiredAngle()), -1, 1);

    driveMotor.set(speed_power);
    angleMotor.set(angle_power);

    updateTotalDegreesTurned();
    updateIncrementMeasurement();
  }

  public void stop() {
    angleMotor.stopMotor();
    driveMotor.stopMotor();
    
    updateIncrementMeasurement();
  }

  public void stopDefensively() {
    update(0.0000001, position.getAngle());
  }

  

}
