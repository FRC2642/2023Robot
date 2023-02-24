// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.TreeSet;
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
    public static final String PARENT_DIRECTORY =  Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner/generatedJSON/";

    /*
     * Creates an empty path
     */
    public PiratePath() {

    }

    /*
     * Creates a path from a JSON file from FRC PathPlanner 2023
     */
    public PiratePath(String fileName) throws JsonProcessingException, IOException {
        Exception e = trySetFromPathPlannerJSON(new File(PARENT_DIRECTORY, fileName));
        if (e != null) {
            e.printStackTrace();
            add(DEFAULT_VALUE);
        }
    }

    public Exception trySetFromPathPlannerJSON(File jsonFile) {
        try {
            JsonNode root = JSON_MAPPER.readTree(jsonFile);
            var pointIterator = root.elements();

            boolean first = true;
            while (pointIterator.hasNext()) {
                var point = pointIterator.next();
                double t = point.get("time").asDouble();
                JsonNode pose = point.get("pose");

                double h = Math.toRadians(point.get("holonomicRotation").asDouble());
                JsonNode translation = pose.get("translation");

                double x = translation.get("x").asDouble();
                double y = translation.get("y").asDouble();

                boolean stop = point.get("velocity").asDouble() == 0.0 && !first;

                // CONVERT TO FEET
                x *= Constants.FOOT_PER_METER;
                y *= Constants.FOOT_PER_METER;

                PiratePoint pt = new PiratePoint(x, y, h, t, stop);
                add(pt);
                first = false;
            }

        } catch (Exception e) {
            return e;
        }
        return null;
    }

    public ArrayList<PiratePath> getSubPaths() {
        ArrayList<PiratePath> paths = new ArrayList<>();
        

        PiratePath current = new PiratePath();
        for (var pt : this) {
            current.add(pt);
            if (pt.stopPoint && pt.time != 0.0) {
                paths.add(current);
                current = new PiratePath();
            
            }
        }

        return paths;
    }

    @Override
    public String toString() {
        ArrayList<String> s = new ArrayList<>();
        for(var pt : this) {
            s.add(pt.toString());
        }
        return String.join(",", s);
    }

    public void fillWithSubPointsEasing(double timeBetweenPts, Easings.Functions interpolationEasing) {

        ArrayList<PiratePoint> points = new ArrayList<>(this);

        for(int i = 0; i < points.size() - 1; i++) {
            PiratePoint current = points.get(i);
            PiratePoint next = points.get(i + 1);

            for(double time = current.time; time < next.time; time += timeBetweenPts) {
                double x = Easings.interpolate(current.position.getX(), next.position.getX(), current.time, next.time, time, interpolationEasing);
                double y = Easings.interpolate(current.position.getY(), next.position.getY(), current.time, next.time, time, interpolationEasing);
                double h = Easings.interpolate(current.heading, next.heading, current.time, next.time, time, interpolationEasing);
                add(new PiratePoint(x, y, h, time, false));
            }
        }
    }

    public double getDuration() {
        return last().time;
    }

    public PiratePoint getFirst(){
        return first();
    }
}
