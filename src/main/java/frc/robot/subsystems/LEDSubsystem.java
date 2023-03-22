// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.mindsensors.*;
public class LEDSubsystem extends SubsystemBase {

  private final CANLight lights;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    this.lights = new CANLight(Constants.LED_PORT);
  }

  public void fade(int start_rgb, int end_rgb) {
    lights.fade(start_rgb, end_rgb);
  }

  public void blink(int seconds) {
    lights.blinkLED(seconds);
  }

  public void startBlackGreenFade() {
    // lights.showRGB(255, 0, 0);

    lights.writeRegister(0, 0.5, 44, 195, 48);
    lights.writeRegister(1, 0.25, 0, 0, 0);

    fade(0,1);

  }

  public void panic() {
    lights.showRGB(255, 0, 0);
    lights.blinkLED(3);
  }
  /*
   * starts an led action between some number of colors
   * @param action Either "flash" or "cycle", specifies action for LEDs to take
   * @param colors 2D array of integers, each row should be 3 integers, each specifying a red, green, and blue value, respectively.
   * @param times Array of doubles that specify the time for which each 
   */
  public void startActionBetweenColors(String action, Color[] colors){
    // if (colors.length > 8 || times.length > 8) {
    //   panic();
    //   System.out.println("You put too many colors in");
    // }

    // if (colors.length != times.length) {
    //   panic();
    //   System.out.println("Length of colors and times don't match");
    // }

    if (colors.length == 0) {
      panic();
      System.out.println("bruh empty color list wdy want me to do");
    }

    for (int ind = 0; ind <= colors.length; ind++) {
      Color color = colors[ind];
      //lights.writeRegister(ind, /*color.time <=1 ? color.time : 1*/ color.time, color.red, color.green, color.blue);
    }

    if (action == "fade") {
      lights.fade(0, colors.length);
    }
    if (action == "cycle") {
      lights.cycle(0, colors.length);
    }
  } 

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
