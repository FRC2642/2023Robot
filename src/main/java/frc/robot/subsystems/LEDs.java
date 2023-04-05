// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends SubsystemBase {
  
  public final static CANdle candle = new CANdle(Constants.LED_PORT);
  private final static Animation rainbow = new RainbowAnimation(1, 0.9, 194);
  private final static Animation yellow = new StrobeAnimation(200, 200, 0, 0, 0.1, 194, 0);
  /** Creates a new LEDs. */
  public LEDs() {
    candle.configLEDType(LEDStripType.GRB);

    rainbowLED();
    //config.
  }


  public static void setLEDCOLOR(int r, int g, int b){
    candle.setLEDs(r, g, b);
  }

  public static void rainbowLED(){
    candle.animate(rainbow);
  }

  public static void strobeYellow(){
    candle.animate(yellow);
  }


  @Override
  public void periodic() {
    //setLEDCOLOR(0, 255, 0);
  }
}
