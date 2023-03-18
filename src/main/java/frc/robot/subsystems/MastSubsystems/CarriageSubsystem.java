// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.IPositionable;
import frc.robot.utils.MathR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

public class CarriageSubsystem extends SubsystemBase implements IPositionable<CarriageSubsystem.CarriagePosition> {

  public static final double FULL_EXTENSION_PER_TICK = 1d / 520d;
  public static final double AT_SETPOINT_THRESHOLD = 10d;

  private final CANSparkMax carriageMotor = new CANSparkMax(Constants.CARRIAGE_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder carriageEncoder = carriageMotor.getEncoder();
  private final PIDController carriagePIDController = new PIDController(1.0, 0, 0);
  private static SparkMaxLimitSwitch bottomLimitSwitch;
  private static SparkMaxLimitSwitch topLimitSwitch;

  private CarriagePosition currentSetPosition = CarriagePosition.RETRACTED;
  private double speedLimit = 1.0;


  public CarriageSubsystem() {
    carriageMotor.setClosedLoopRampRate(0.0);
    carriageMotor.setOpenLoopRampRate(0.0);
    carriageMotor.setInverted(false);
    carriageEncoder.setPositionConversionFactor(FULL_EXTENSION_PER_TICK);
    carriagePIDController.setTolerance(AT_SETPOINT_THRESHOLD);
    bottomLimitSwitch = carriageMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    topLimitSwitch = carriageMotor.getForwardLimitSwitch(Type.kNormallyOpen);
  }

  @Override
  public void set(CarriagePosition pos) {
    set(carriagePIDController.calculate(getCarriageExtension(), pos.extension));
    currentSetPosition = pos;
  }

  //Positive = up
  public void set(double speed) {
    currentSetPosition = CarriagePosition.MANUAL;
    carriageMotor.set(MathR.limit(speed, -speedLimit, speedLimit));
  }
  
  public static boolean isCarriageUp() {
    return topLimitSwitch.isPressed();
  }

  public static boolean isCarriageDown() {
    return bottomLimitSwitch.isPressed();
  }

  public double getCarriageExtension() {
    return carriageEncoder.getPosition();
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
  public void setSpeedLimit(double max) {
    speedLimit = max;
  }

  @Override
  public double getSpeedLimit() {
    return speedLimit;
  }

  @Override
  public void setRampRate(double rampRate) {
    carriageMotor.setOpenLoopRampRate(rampRate);
  }

  @Override
  public double getRampRate() {
    return carriageMotor.getOpenLoopRampRate();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Carriage Up", isCarriageUp());
    SmartDashboard.putBoolean("Carriage Down", isCarriageDown());
  }
  
  public enum CarriagePosition {
    EXTENDED(1),
    RETRACTED(0),
    MANUAL(-1);

    public final double extension;
    private CarriagePosition(double extension) {
      this.extension = extension;
    }
  }

}
