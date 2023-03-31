// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public LEDs() {}

  public static void setLEDCOLOR(int r, int g, int b){
    final CANdle candle = new CANdle(0);
    candle.setLEDs(r, g, b);
  }

  public static void setLEDANIMATION(int id, double speed, int numLed, int ledOffset){
    final Animation animation = new Animation(id, speed, numLed, ledOffset);
    }
  }
  @Override
  public void periodic() {
    
  }
}
