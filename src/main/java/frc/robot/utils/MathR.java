// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class MathR {

    public static double getDistanceToAngleRadians(double current, double desired) {
        return modulo(((desired) - (current)) + Math.PI, Math.PI * 2.0) - Math.PI;
    }

    public static double modulo(double x, double y) {
        if (x >= 0)
            return x % y;
        else {
            while (x < 0)
                x += y;
            return x % y;
        }
    }

    public static double limit(double input, double min, double max) {
        if (input < min) return min;
        if (input > max) return max;
        return input;
    }

    public static double ticksToDegrees(double ticks, double ticksPerRev) {
        return ((360 / ticksPerRev) * ticks);
    }

    public static double halfOptimize(double current, double setpoint, double fullRotationNum) {
        // Crossing 0-360 line counterclockwise
        if ((current + fullRotationNum) - setpoint <= fullRotationNum / 2) {
            return (current + fullRotationNum);
        }
        // Crossing 0-360 line clockwise
        else if ((current - fullRotationNum) - setpoint >= -fullRotationNum / 2) {
            return (current - fullRotationNum);
        }
        // Not passing line
        else {
            return current;
        }

    }

    public static double lerp(double outputMin, double outputMax, double inputMin, double inputMax, double input) {
        return outputMin + (outputMax - outputMin)*(input - inputMin)/(inputMax - inputMin);
    }

    
}
