// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.swerve.SwerveModuleInfo;
import static frc.robot.subsystems.swerve.SwerveModules.ModuleLocation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {

    public static final Alliance ALLIANCE = DriverStation.getAlliance();

    // Ratios
    public static final double FEET_PER_DISPLACEMENT = 6.1078e-5;//6.52131382735e-5;
    public static final double MODULE_ANGLE_KP = 0.00524;
    public static final double FOOT_PER_METER = 3.28084;

    // Controlers
    public static final int DRIVE_CONTROL_PORT = 0;
    public static final int AUX_CONTROL_PORT = 1;

    // Swerve
    public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(8, 7, 14, 360, 62.2, 1, -1, ModuleLocation.FRONT_RIGHT);
    public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(2, 1, 11, 360, 66.7, 1, 1, ModuleLocation.FRONT_LEFT);
    public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(6, 5, 13, 360, 287.9, -1, -1, ModuleLocation.BACK_RIGHT);
    public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(4, 3, 12, 360, 0.3, -1, 1, ModuleLocation.BACK_LEFT);

    // Mast
    public static final int MAIN_SLIDER_MOTOR = 21;
    public static final int CARRIAGE_MOTOR = 22;
    public static final int SHOULDER_MOTOR_1 = 23;
    public static final int SHOULDER_MOTOR_2 = 23;

    // Claw
    public static final int WRIST_MOTOR = 24;
    public static final int GRIPPER_INTAKE_MOTOR = 25;

    // Gripper
    public static final int GRIPPER_SOLENOID_CHANNEL = 0;

    // Other
    public static final String TEST_PATH = "New New New Path.wpilib.json";

}
