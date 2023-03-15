// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import frc.robot.utils.VectorR;

/** Add your docs here. */
public class PiratePoint implements Comparable<PiratePoint> {
    
    public final VectorR position;
    public final double heading;
    public final double time;
    public final boolean stopPoint;

    public PiratePoint(double x, double y, double heading, double time, Boolean stopPoint) {
        position = VectorR.fromCartesian(x, y);
        this.heading = heading;
        this.time = time;
        this.stopPoint = stopPoint;
    }

    @Override
    public int compareTo(PiratePoint arg0) {
        return Double.compare(time, arg0.time);
    }

    @Override
    public String toString() {
        return "{[pos: " + position.toString() + "ft] [t: " + time + "s] [head: " + Math.toDegrees(heading) + "deg]}";
    }
}
