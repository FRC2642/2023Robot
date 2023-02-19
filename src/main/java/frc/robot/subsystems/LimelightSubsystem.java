// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DataStreamFilter;

public class LimelightSubsystem extends SubsystemBase {

 // private double x;
  //private double y;
  //private double area;
  private static NetworkTable table;

  
 // private final DataStreamFilter filterX = new DataStreamFilter(10, 2d);
 // private final DataStreamFilter filterY = new DataStreamFilter(10, 2d);



  public double x;
  public double y;
  public double a;
  public double zrot;
  public boolean isDetection = false;

  
  private void updateLimelightMeasurements() {
    double[] transform = table.getEntry("botpose").getDoubleArray(new double[6]);

    isDetection = table.getEntry("tv").getDouble(0.0) == 1.0;
    a = table.getEntry("ta").getDouble(0);
    x = transform[0] * Constants.FOOT_PER_METER + 27.0416;
    y = transform[1] * Constants.FOOT_PER_METER + 13.2916;
    zrot = transform[5];
  }

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    delta_t_timer.start();
  }

  private final DataStreamFilter diffFilterX = new DataStreamFilter(20);
  private final DataStreamFilter diffFilterY = new DataStreamFilter(20);
  private double confidence = -1;

  private Timer delta_t_timer = new Timer();
  public double delta_x;
  public double delta_y;

  public double getConfidence() {
    return confidence;
  }


  @Override
  public void periodic() {
  //  if (delta_t == 0) return;

    double delta_t = delta_t_timer.get() / 0.022;

    delta_x = x;
    delta_y = y;

   updateLimelightMeasurements();

    delta_x -= x;
    delta_y -= y;

    //System.out.println("filter: " + diffFilterX.getRunningAvg());
   // SmartDashboard.putNumber("timer", delta_t);
   //System.out.println("dx: " + delta_x + "dy: " + delta_y + "dt: " + delta_t + "x: " + x + "y: " + y + "");
   if (delta_t != 0 && isDetection && delta_x != 0 && delta_y != 0){
      diffFilterX.calculate(Math.abs(delta_x/delta_t));
      diffFilterY.calculate(Math.abs(delta_y/delta_t));
      
      confidence = isDetection ? 2/(diffFilterX.getRunningAvg() + diffFilterY.getRunningAvg()) : -1;
      if (Double.isInfinite(confidence)) confidence = -3;

   }
   else if (!isDetection) confidence = -1;
   //else if (delta_x != 0 && delta_y != 0) confidence = -2;
   delta_t_timer.reset();

    //SmartDashboard.putBoolean("detections", isDetection);
    SmartDashboard.putNumber("confidence", confidence);
    //SmartDashboard.putNumber("x", x);
    //SmartDashboard.putNumber("y", y);
    //SmartDashboard.putNumber("z rot", zrot);
  }
}