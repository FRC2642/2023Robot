// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClawSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  private final CANSparkMax wristMotor = new CANSparkMax(24, MotorType.kBrushed);
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder(Type.kQuadrature, 4);
  
  private final PIDController wristPIDController = new PIDController(0.2, 0, 0);
  private WristPosition currentSetPosition = WristPosition.MANUAL;
  
  public enum WristPosition {
    MANUAL(-1),
    HORIZONTAL1(180),
    HORIZONTAL2(360),
    VERTICAL1(90),
    VERTICAL2(270);

    public double angle;

    private WristPosition(double angle) {
      this.angle = angle;
    }
  }
  /** Creates a new ClawWristSubsystem. */
  public ClawWristSubsystem() {
    wristEncoder.setPositionConversionFactor(DEGREES_PER_TICK);
  }

  public double getWristAngle() {
    return wristEncoder.getPosition();
  }

  public void resetWristEncoder() {
    wristEncoder.setPosition(180.0);
  }

  public void set(double speed) {
    currentSetPosition = WristPosition.MANUAL;
    wristMotor.set(speed);
  }

  public void set(WristPosition pos) {
    currentSetPosition = pos;
    wristMotor.set(wristPIDController.calculate(getWristAngle(), pos.angle));
  }

  public boolean atSetPosition() {
    return wristPIDController.atSetpoint();
  }

  public WristPosition getSetPosition() {
    return currentSetPosition;
  }
      
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
  }
}
