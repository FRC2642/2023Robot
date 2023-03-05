// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModules;
import frc.robot.utils.TimedVectorDerivative;
import frc.robot.utils.VectorR;

public class DriveSubsystem extends SubsystemBase{
  

  // COMPONENTS
  public final SwerveModules modules;
  private static AHRS gyro;
  

  // POSITION TRACKING
  private static VectorR displacement;
  private static VectorR velocity;
  private static TimedVectorDerivative acceleration;
  private static TimedVectorDerivative jerk;

  //OTHER
  public boolean defenseActivated = true;
  private static double yawOffsetDegrees = 0;

  public  DriveSubsystem() {
    modules = new SwerveModules(
        new SwerveModule(Constants.FRONT_RIGHT), new SwerveModule(Constants.FRONT_LEFT),
        new SwerveModule(Constants.BACK_RIGHT), new SwerveModule(Constants.BACK_LEFT));

    gyro = new AHRS();
    gyro.reset();

    displacement = new VectorR();
    velocity = new VectorR();
    acceleration = new TimedVectorDerivative(velocity);
    jerk = new TimedVectorDerivative(acceleration);

  }

  /*
   * SYSTEM STANDARD FOLLOWS COORDINATE PLANE STANDARD
   * positive (+) = forwards/left/left turn CCW
   * negative (-) = backwards/right/right turn CW
   * velocity magnitude (0-1) 1:fastest 0:stopped
   * turn (0-1) 
   * NOTE: the speed of any wheel can reach a maximum of turn + |velocity|
   */
  public void move(VectorR velocity, double turn) {

    DriveSubsystem.velocity.setFromCartesian(0, 0);

    VectorR directionalPull = velocity.clone();
    directionalPull.rotate(Math.toRadians(-getYawDegrees()));

    for (SwerveModule module : modules) {

      double moduleTangent = module.position.getAngle() + Math.toRadians(90);
      VectorR rotationalPull = VectorR.fromPolar(turn, moduleTangent);

      VectorR wheelPull = VectorR.addVectors(directionalPull, rotationalPull);
      double speed = wheelPull.getMagnitude();
      double angle = wheelPull.getAngle();

      module.update(speed, angle);
      

      //position tracking
      var increment = module.getPositionIncrement();
      increment.mult(1d / 4d);
      increment.rotate(Math.toRadians(getYawDegrees()));
      DriveSubsystem.displacement.add(increment);

      var velocityMeasured = module.getVelocity();
      velocityMeasured.mult(1d / 4d);
      velocityMeasured.rotate(Math.toRadians(getYawDegrees()));
      DriveSubsystem.velocity.add(velocityMeasured);
    }
    DriveSubsystem.acceleration.update();
    DriveSubsystem.jerk.update();
  }
  

  /*
   * public void debugWheelDirections(double angle) {
   * modules.frontRight.update(0.25, angle);
   * modules.frontLeft.update(0.25, angle);
   * modules.backRight.update(0.25, angle);
   * modules.backLeft.update(0.25, angle);
   * }
   */

  
  public void setDefensiveMode(boolean defensive) {
    defenseActivated = defensive;
  }

  public VectorR getDisplacement(){
    return displacement;
  }


  public static void resetDisplacement(VectorR v) {
    displacement.setFromCartesian(v.getX(), v.getY());
  }


  public void setDefenseMode(boolean activated){
    defenseActivated = activated;
  }

  public void stop() {
    for (SwerveModule module : modules) {
      if (defenseActivated)
        module.stopDefensively();
      else
        module.stop();
    }

    velocity.setFromCartesian(0, 0);
    DriveSubsystem.acceleration.update();
    DriveSubsystem.jerk.update();
  }

  // POSITION DATA
  public static VectorR getRelativeFieldPosition() {
    return DriveSubsystem.displacement.clone();
  }

  public static VectorR getRelativeVelocity() {
    return DriveSubsystem.velocity.clone();
  }


  public static VectorR getRelativeAcceleration() {
    return DriveSubsystem.acceleration.clone();
  }

  public static VectorR getRelativeJerk() {
    return DriveSubsystem.jerk.clone();
  }
  
  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public static double getYawDegrees() {
    return -1 * gyro.getYaw() + yawOffsetDegrees;
  }

  public void resetEncoders() {
    for (var mod : modules)
      mod.resetDriveEncoder();
  }

  //+ LEFT
  public static double getRoll(){
    return gyro.getRoll();
  }

  public static double getPitch(){
    return gyro.getPitch();
  }

  public static void resetGyro(double yawDegrees) {
    gyro.reset();
    yawOffsetDegrees = yawDegrees;
  }



  @Override
  public void periodic() {
    // modules.debugSmartDashboard();
    modules.debugSmartDashboard();

    SmartDashboard.putNumber("pitch:",getPitch());
    SmartDashboard.putNumber("roll:",getRoll());
    

    //SmartDashboard.putNumber("x field", displacement.getX());
    //SmartDashboard.putNumber("y field", displacement.getY());

    SmartDashboard.putNumber("gyro", getYawDegrees());

    //SmartDashboard.putNumber("distance [ft]", getRelativeFieldPosition().getMagnitude());
    //SmartDashboard.putNumber("speed [ft/sec]", getRelativeVelocity().getMagnitude());
    // SmartDashboard.putNumber("angle [degrees]",
    // Math.toDegrees(getRelativeFieldPosition().getAngle()));
    // SmartDashboard.putNumber("speed [ft/s]",
    // getRelativeVelocity().getMagnitude());
    // SmartDashboard.putNumber("accell [ft/s^2]",
    // getRelativeAccelleration().getMagnitude());
    // SmartDashboard.putNumber("jerk [ft/s^3]", getRelativeJerk().getMagnitude());
  }
}
