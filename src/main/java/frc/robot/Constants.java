// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.swerve.SwerveModuleInfo;

public final class Constants {
    public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(8, 7, 14, 360, 62.2, 1, -1);
    public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(2, 1, 11, 360, 66.7, 1, 1);
    public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(6, 5, 13, 360, 287.9, -1, -1);
    public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(4, 3, 12, 360, 0.3, -1, 1);
    public static final int DRIVE_CONTROL_PORT = 0;
    public static final int AUX_CONTROL_PORT = 1;
    public static final double FEET_PER_DISPLACEMENT = 6.52131382735e-5;
    public static final double MODULE_ANGLE_KP = 0.3;
    public static final String TEST_PATH = "Backwards.wpilib.json";
    public static final double FOOT_PER_METER = 3.28084;
    public static final int GRIPPER_SOLENOID_CHANNEL = 0;
}
