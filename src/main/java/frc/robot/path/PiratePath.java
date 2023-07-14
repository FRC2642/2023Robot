// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import java.io.Console;
import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.SortedSet;
import java.util.TreeSet;

import org.opencv.core.Point;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.utils.Easings;

/**
 * Represents a field path for a swerve robot, contains helper functions for
 * reading and building paths. CANNOT BE CHANGED WHILE READING
 */
public class PiratePath extends TreeSet<PiratePoint> {

    public static final ObjectMapper JSON_MAPPER = new ObjectMapper();
    public static final PiratePoint DEFAULT_VALUE = new PiratePoint(0, 0, 0, 0, false);
    public static final String PARENT_DIRECTORY = Filesystem.getDeployDirectory().getAbsolutePath()
            + "/pathplanner/generatedJSON/";

    public final boolean allianceDependent;
    public String name;

    /*
     * Creates an empty path
     */
    public PiratePath(boolean allianceDependent) {
        this.allianceDependent = allianceDependent;
        name = "unnamed";
    }

    /*
     * Creates a path from a JSON file from FRC PathPlanner 2023
     */
    public PiratePath(String name) {
        this.name = name;
        Exception e = trySetFromPathPlannerJSON(new File(PARENT_DIRECTORY, name + ".wpilib.json"));
        if (e != null) {
            e.printStackTrace();
            add(DEFAULT_VALUE);
        }
        allianceDependent = true;
    }

    public Exception trySetFromPathPlannerJSON(File jsonFile) {
       
        try {
            JsonNode root = JSON_MAPPER.readTree(jsonFile);
            var pointIterator = root.elements();
            boolean first = true;
            while (pointIterator.hasNext()) {
                var point = pointIterator.next();
                JsonNode pose = point.get("pose");
                JsonNode translation = pose.get("translation");

                double t = point.get("time").asDouble();
                double x = (Constants.FIELD_X) - (translation.get("x").asDouble() * Constants.FOOT_PER_METER);
                double y = (Constants.FIELD_Y) - (translation.get("y").asDouble() * Constants.FOOT_PER_METER);
                double h = point.get("holonomicRotation").asDouble() + 180;
                boolean stop = point.get("velocity").asDouble() == 0.0 && !first;

                PiratePoint pt = new PiratePoint(x, y, h, t, stop);
                add(pt);
                first = false;
            }
        } catch (Exception e) {
            return e;
        }
        return null;
    }

    public PiratePath getBlueAlliance() {

        PiratePath bluePath = new PiratePath(true);
        bluePath.name = "BLUE " + this.name;

        for (var pt : this) {
            double Y = pt.position.getY();
            var newPt = pt.clone();
            newPt.position.setY(Constants.FIELD_Y - Y);
            //newPt.
            bluePath.add(newPt);
        }

        return bluePath;
    }

    public ArrayList<PiratePath> getSubPaths() {
        ArrayList<PiratePath> paths = new ArrayList<>();

        PiratePath current = new PiratePath(true);
        int index = 0;
        for (var pt : this) {
            current.add(pt);
            if (pt.stopPoint && pt.time != 0.0) {
                current.name = this.name + "[" + index + "]";
                paths.add(current);
                current = new PiratePath(true);
                index++;
            }
        }

        return paths;
    }

    @Override
    public String toString() {
        ArrayList<String> s = new ArrayList<>();
        for (var pt : this) {
            s.add(pt.toString());
        }
        return name + "\n" + String.join("\n", s);
    }

    public void print() {
        System.out.println("--------------------------------------------------------------------------------------------------------------");
        System.out.println(toString());
        System.out.println("--------------------------------------------------------------------------------------------------------------");
    }
    
    public static void print(ArrayList<PiratePath> paths) {
        for (var p : paths) {
            p.print();
        } 
    }
    
    public static void print(ArrayList<PiratePath> paths, int index) {
        paths.get(index).print();
    }

    public void fillWithSubPointsEasing(double timeBetweenPts, Easings.Functions interpolationEasing) {

        ArrayList<PiratePoint> points = new ArrayList<>(this);

        for (int i = 0; i < points.size() - 1; i++) {
            PiratePoint current = points.get(i);
            PiratePoint next = points.get(i + 1);

            for (double time = current.time; time < next.time; time += timeBetweenPts) {
                double x = Easings.interpolate(current.position.getX(), next.position.getX(), current.time, next.time,
                        time, interpolationEasing);
                double y = Easings.interpolate(current.position.getY(), next.position.getY(), current.time, next.time,
                        time, interpolationEasing);
                double h = Easings.interpolate(current.heading, next.heading, current.time, next.time, time,
                        interpolationEasing);
                add(new PiratePoint(x, y, h, time, false));
            }
        }
    }

    public double getLastTime() {
        return last().time;
    }

    public PiratePoint getFirst() {
        return first();
    }
}
