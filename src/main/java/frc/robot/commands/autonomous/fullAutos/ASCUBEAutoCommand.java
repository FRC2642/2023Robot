// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.fullAutos;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.path.PiratePath;

// 
public class ASCUBEAutoCommand extends SequentialCommandGroup {
  
  public ASCUBEAutoCommand() {
    PiratePath path;
    try {
      path = new PiratePath("ASCUBE.wpilib.json");
    } 
    catch (IOException e) {
      e.printStackTrace();
    }
    


    addCommands();
  }
}
