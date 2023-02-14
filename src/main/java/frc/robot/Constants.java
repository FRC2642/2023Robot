// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.swerve.SwerveModuleInfo;

public final class Constants {
    public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(1, 2, 14, 3.16, 2.948, 1, -1);
    public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(5, 6, 11, 3.29, 2.150, 1, 1);
    public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(3, 4, 13, 3.318, 1.680, -1, -1);
    public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(7, 8, 12, 3.293, 0.995, -1, 1);
    public static final int DRIVE_CONTROL_PORT = 0;
    public static final int AUX_CONTROL_PORT = 1;
    public static final double FEET_PER_DISPLACEMENT = 0.12765957446;
    public static final double MODULE_ANGLE_KP = 0.3;
    public static final String TEST_PATH = "Backwards.wpilib.json";
    public static final double FOOT_PER_METER = 3.28084;
}
