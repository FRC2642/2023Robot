// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends SubsystemBase {
  
  public final static CANdle candle = new CANdle(Constants.LED_PORT);
   /** Creates a new LEDs. */
  public LEDs() {
    candle.configLEDType(LEDStripType.GRB);

  }

  public void animateLEDs(LEDPattern pattern){
    candle.animate(pattern.currentAnimation);
  }



  public enum LEDPattern{
    RAINBOW(new RainbowAnimation(1, 0.9, Constants.LED_LENGTH)),
    STROBE_YELLOW(new LarsonAnimation(200, 200, 0, 0, 0.1, Constants.LED_LENGTH, BounceMode.Front, 1)),
    STROBE_PURPLE(new LarsonAnimation(160, 32, 240, 0, 0.1, Constants.LED_LENGTH, BounceMode.Front, 1)),
    SOLID_GREEN(new ColorFlowAnimation(0, 255, 0)),
    SOLID_RED(new ColorFlowAnimation(255, 0, 0)),
    SOLID_BLUE(new ColorFlowAnimation(0, 0, 255));


    public Animation currentAnimation;
    private LEDPattern(Animation animation){
      this.currentAnimation = animation;
    }
  }


  @Override
  public void periodic() {
    //setLEDCOLOR(0, 255, 0);
  }
}
