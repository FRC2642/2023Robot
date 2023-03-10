// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.HashMap;
import java.util.Iterator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** Makes it easier to access swerve modules */
public class SwerveModules implements Iterable<SwerveModule> {

  // MODULES
  public final SwerveModule frontRight;
  public final SwerveModule frontLeft;
  public final SwerveModule backRight;
  public final SwerveModule backLeft;

  private final HashMap<ModuleLocation, SwerveModule> modules = new HashMap<>();

  public SwerveModules(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backRight, SwerveModule backLeft) {
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.backRight = backRight;
    this.backLeft = backLeft;

    modules.put(ModuleLocation.FRONT_RIGHT, frontRight);
    modules.put(ModuleLocation.FRONT_LEFT, frontLeft);
    modules.put(ModuleLocation.BACK_RIGHT, backRight);
    modules.put(ModuleLocation.BACK_LEFT, backLeft);
  }

  // get a module based on an enum value
  public SwerveModule get(ModuleLocation module) {
    return modules.get(module);
  }

  public enum ModuleLocation {
    FRONT_RIGHT,
    FRONT_LEFT,
    BACK_RIGHT,
    BACK_LEFT
  }

   
  public void debugSmartDashboard() {
   /*  SmartDashboard.putNumber("FR", frontRight.driveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL", frontLeft.driveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("BR", backRight.driveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("BL", backLeft.driveMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("FR heading", Math.toDegrees(frontRight.getWheelHeadingRadians()));
    SmartDashboard.putNumber("FL heading", Math.toDegrees(frontLeft.getWheelHeadingRadians()));
    SmartDashboard.putNumber("BR heading", Math.toDegrees(backRight.getWheelHeadingRadians()));
    SmartDashboard.putNumber("BL heading", Math.toDegrees(backLeft.getWheelHeadingRadians()));


    SmartDashboard.putNumber("FL Turn Encoder", frontLeft.getRelativeTurnEncoderValue());*/
  } 

  // get a list of the modules for looping through (iterating)
  @Override
  public Iterator<SwerveModule> iterator() {
    return modules.values().iterator();
  }

}