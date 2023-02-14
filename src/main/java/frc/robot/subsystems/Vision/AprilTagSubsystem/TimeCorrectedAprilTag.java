package frc.robot.subsystems.Vision.AprilTagSubsystem;

import java.security.Timestamp;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimesliceRobot;
import frc.robot.subsystems.Vision.VisionHelpers.AprilTagConfig;

/**
     * This is an extension of the wpilib AprilTag class that adds the
     * ability to place the active position of the april tag in the class
     * to pass it around
     */
    public class TimeCorrectedAprilTag extends AprilTag{
        public Pose3d relativePose;
        double timeStamp;
        public double ambiguity;
        NetworkTable table;
        Double maxHistTime = 1.0;
        //  Timestamp, <pose, ambiguity>
        TreeMap<Double, Map<Pose3d, Double>> historicalValues = new TreeMap<Double, Map<Pose3d, Double>>();

        public TimeCorrectedAprilTag(AprilTagConfig config, NetworkTable table) {
            super(config.getId(), config.getPose());
            this.table = table;
        }

        public void updatePosition(Pose3d relativePose, double timeStamp, double ambiguity) {
            Map positions = new HashMap<Pose3d, Double>();
            positions.put(relativePose, ambiguity);
            historicalValues.put(timeStamp, positions);
            historicalValues.remove(historicalValues.firstKey());
        }
        


    }