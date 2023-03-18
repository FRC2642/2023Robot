// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystems.ClawGripperSubsystem;
import frc.robot.subsystems.interfaces.IPositionable;
import frc.robot.utils.MathR;

public class SliderSubsystem extends SubsystemBase implements IPositionable<SliderSubsystem.SliderPosition> {

  public static final double FULL_EXTENSION_PER_TICK = 1d / 240d;
  public static final double AT_SETPOINT_THRESHOLD = 0.1;

  private final CANSparkMax sliderMotor = new CANSparkMax(Constants.MAIN_SLIDER_MOTOR, MotorType.kBrushless);
  private final Solenoid brake = ClawGripperSubsystem.pneumatics.makeSolenoid(2);
  private final RelativeEncoder sliderEncoder = sliderMotor.getEncoder();

  private final PIDController sliderPIDController = new PIDController(0.5, 0, 0);
  private SliderPosition currentSetPosition = SliderPosition.RETRACTED;
  private double speedLimit = 1;

  public SliderSubsystem() {
    sliderEncoder.setPositionConversionFactor(FULL_EXTENSION_PER_TICK);
    sliderMotor.setOpenLoopRampRate(0.0);
    sliderMotor.setClosedLoopRampRate(0.0);
    sliderMotor.setInverted(false);
    sliderPIDController.setTolerance(AT_SETPOINT_THRESHOLD);
  }

  public void set(SliderPosition pos) {
    double speed = sliderPIDController.calculate(getSliderExtension(), pos.extension);
    
    if (!atSetPosition()) set(speed);
    else set(0.0);

    currentSetPosition = pos;
  }

  public void set(double speed) {
    currentSetPosition = SliderPosition.MANUAL;
    
   // if (speed == 0.0) brake.set(true);
   // else brake.set(false);

    sliderMotor.set(MathR.limit(speed, -speedLimit, speedLimit));
  }
  
  public void resetSliderEncoder(SliderPosition pos) {
    sliderEncoder.setPosition(pos.extension);
  }

  public double getSliderExtension(){
    return sliderEncoder.getPosition();
  }

  @Override
  public SliderPosition getSetPosition() {
    return currentSetPosition;
  }

  @Override
  public boolean atSetPosition() {
    return sliderPIDController.atSetpoint();
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
    sliderMotor.setOpenLoopRampRate(rampRate);
  }

  @Override
  public double getRampRate() {
    return sliderMotor.getOpenLoopRampRate();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Slider Extension", getSliderExtension());
  }
  
  public enum SliderPosition {
    MANUAL(-1),
    RETRACTED(0),
    PARTIALLY(0.7),
    EXTENDED(1);

    public double extension;
    private SliderPosition(double extension) {
      this.extension = extension;
    }
  }
}
