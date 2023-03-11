// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


/** Wrapper class for vectors */
public class VectorR implements Cloneable {

    // vector data
    private double x, y;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    // constructor
    public VectorR() {
        x = 0;
        y = 0;
    }

    // helper functions

    // adds a vector to this vector
    public void add(VectorR... vector) {
        for (var vec : vector) {
            x += vec.x;
            y += vec.y;
        }
    }

    public void sub(VectorR vector) {
        x -= vector.x;
        y -= vector.y;
    }

    public void pow(double val) {
        setMagnitude(Math.pow(getMagnitude(), val));
    }

    public void mult(double val) {
        setMagnitude(getMagnitude() * val);
    }

    public void div(double val){
        setMagnitude(getMagnitude() / val);
    }

    public double dot(VectorR v2) {
        return v2.getX() * x + v2.getY() * y;
    }

    public static VectorR addVectors(VectorR... vectors) {
        VectorR v3 = new VectorR();
        
        for (VectorR vector : vectors){
            v3.add(vector);
        }
        return v3;
    }

    public static VectorR subVectors(VectorR... vectors) {
        VectorR v3 = vectors[0].clone();
        
        for (int i = 1; i < vectors.length; i++){
            v3.sub(vectors[i]);
        }
        return v3;
    }
    
    public static VectorR subVectors(VectorR v1, VectorR v2) {
        var v3 = v1.clone();
        v3.sub(v2);
        return v3;
    }

    public void setFromPolar(double distance, double angle) {
        x = distance * Math.cos(angle);
        y = distance * Math.sin(angle);
    }

    public void setFromCartesian(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setMagnitude(double mag) {
        setFromPolar(mag, getAngle());
    }

    public void setAngle(double rad) {
        setFromPolar(getMagnitude(), rad);
    }

    public void setX(double val) {
        x = val;
    }

    public void setY(double val) {
        y = val;
    }

    public void setFrom(VectorR v) {
        x = v.x;
        y = v.y;
    }

    // ALWAYS NEGATIVE MAGNITUDE
    public double getTerminalMagnitude() {
        return getMagnitude() * -1;
    }

    public double getTerminalAngle() {
        return getAngle() + Math.PI;
    }

    @Override
    public VectorR clone() {
        return fromCartesian(x, y);
    }

    public static VectorR fromPolar(double distance, double radians) {
        VectorR v = new VectorR();
        v.setFromPolar(distance, radians);
        return v;
    }

    public static VectorR fromCartesian(double x, double y) {
        VectorR v = new VectorR();
        v.setFromCartesian(x, y);
        return v;
    }

    public void rotate(double angle) {
        setAngle(getAngle() + angle);
    }

    public static boolean compareVectors(VectorR vector1, VectorR vector2){
        boolean magCheck = false;
        boolean angleCheck = false;

        //0.25 speed auto
        //if (Math.abs(vector1.getMagnitude() - vector2.getMagnitude()) <= 1){
        //    magCheck = true;
        //}
        //0.5 speed auto
        //if (Math.abs(vector1.getMagnitude() - vector2.getMagnitude()) <= 2){
        //    magCheck = true;
        //}
        //0.75 speed auto
        if (Math.abs(vector1.getMagnitude() - vector2.getMagnitude()) <= 5){
            magCheck = true;
        }

        /*if (Math.abs(MathR.halfOptimize(vector1.getAngle(), vector2.getAngle(), Math.toRadians(360))-vector2.getAngle()) <= Math.toRadians(10)){
            angleCheck = true;
        }*/

        if (Math.abs(vector1.getAngle() - vector2.getAngle()) <= 5){
            angleCheck = true;
        }
        
        if (magCheck && angleCheck){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public String toString() {
        return "<" + x + "," + y + ">";
    }
}