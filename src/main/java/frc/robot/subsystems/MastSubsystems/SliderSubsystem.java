// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MastSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.IPositionable;

public class SliderSubsystem extends SubsystemBase implements IPositionable<SliderSubsystem.SliderPosition> {

  public static final double FULL_EXTENSION_PER_TICK = 1d / 240d;

  private final CANSparkMax sliderMotor = new CANSparkMax(Constants.MAIN_SLIDER_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder sliderEncoder = sliderMotor.getEncoder();

  private final PIDController sliderPIDController = new PIDController(0.05, 0, 0);
  private SliderPosition currentSetPosition = SliderPosition.RETRACTED;

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

  public void resetSliderEncoder() {
    sliderEncoder.setPosition(0.0);
  }

  public SliderSubsystem() {
    sliderEncoder.setPosition(0.0);
    sliderEncoder.setPositionConversionFactor(FULL_EXTENSION_PER_TICK);
  }

  public void set(SliderPosition pos) {
    currentSetPosition = pos;
    sliderPIDController.setSetpoint(pos.extension);
    sliderMotor.set(sliderPIDController.calculate(getSliderExtension()));
  }

  public void set(double speed) {
    currentSetPosition = SliderPosition.MANUAL;
    sliderMotor.set(speed);
  }

  public double getSliderExtension(){
    return sliderEncoder.getPosition();
  }

  @Override
  public boolean atSetPosition() {
    return sliderPIDController.atSetpoint();
  }

  @Override
  public SliderPosition getSetPosition() {
    return currentSetPosition;
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Slider Extension", getSliderExtension());
  }
}
