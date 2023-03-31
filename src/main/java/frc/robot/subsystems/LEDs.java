// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends SubsystemBase {
  
  public final static CANdle candle = new CANdle(Constants.LED_PORT);
  
  /** Creates a new LEDs. */
  public LEDs() {}


  public static void setLEDCOLOR(int r, int g, int b){
    candle.setLEDs(r, g, b);
  }


  @Override
  public void periodic() {
    
  }
}
