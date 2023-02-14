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
import frc.robot.subsystems.Vision.VisionHelpers.Tuple;

/**
 * This is an extension of the wpilib AprilTag class that adds the
 * ability to place the active position of the april tag in the class
 * to pass it around
 */
public class TimeCorrectedAprilTag extends AprilTag {
    NetworkTable table;
    double maxHist= 1.0;
    double ambiguityThreshold = 0.15;
    //   Ambiguity,    <pose, timestamp>
    TreeMap<Double, Tuple<Pose3d, Double>> historicalValues = new TreeMap<Double, Tuple<Pose3d, Double>>();

    public TimeCorrectedAprilTag(AprilTagConfig config, NetworkTable table) {
        super(config.getId(), config.getPose());
        this.table = table;
    }

    public void updatePosition(Pose3d relativePose, double timeStamp, double ambiguity) {
        Tuple<Pose3d, Double> positions = new Tuple<>(relativePose, timeStamp);
        //TODO: need to check how ambiguity is reported
        //this is just a sketch
        //TODO: find a way to detect major wheel slippage
        if (ambiguity < ambiguityThreshold) {
            historicalValues.put(timeStamp, positions);
        } 
        //Remove values older than allowed
        for (Map.Entry<Double, Tuple<Pose3d, Double>> entry : historicalValues.entrySet()) {
            Double time = entry.getValue().getSecond();
            if (time > maxHist) {
                historicalValues.remove(entry.getKey());
            }
        }

    }

}