package frc.robot.subsystems.Vision.AprilTagSubsystem;

import java.security.Timestamp;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.PolynomialRegression;
import frc.lib.math.MathHelpers.Tuple;
import frc.robot.subsystems.Vision.VisionHelpers.AprilTagConfig;

import static frc.robot.Constants.*;


public class TimeCorrectedAprilTag extends AprilTag {
    NetworkTable table;
    final double maxHist = 1.0;
    //TODO: move ambiguity threshold to constants
    final double ambiguityThreshold = (0.5 / 100.0) * ((1600.0 + 1200.0) / 2.0);
    PolynomialRegression xyStdDevmodel;
    PolynomialRegression thetaStdDevModel;
    double xyStdDev;
    double thetaStdDev;
    double tagDistance;
    double error;

    //      Error,       <pose, timestamp>
    TreeMap<Double, Tuple<Pose3d, Double>> historicalValues = new TreeMap<Double, Tuple<Pose3d, Double>>();

    public TimeCorrectedAprilTag(AprilTagConfig config, PolynomialRegression xyStdDevModel, PolynomialRegression thetaStdDevModel) {
        super(config.id, config.pose);
        this.xyStdDevmodel = xyStdDevModel;
        this.thetaStdDevModel = thetaStdDevModel;
    }

    public void updatePosition(Pose3d relativePose, double timeStamp, double ambiguity) {
        var positions = new Tuple<>(relativePose, timeStamp);
        //this is just a sketch
        //TODO: find a way to detect major wheel slippage
        //NOTE: the standard deviation and ambiguity are probably redundant and I'm not sure that we
        //actually need to use standard deviation as it is only a prediction of error. I want to see if multiplying
        //ambiguity times the standard deviations or the normal distance is superior since ambiguity is just a pixel
        //measurement of error.
        //TODO: Determine if ambiguity (aka reprojection error) comes out as a pixel value or something else
        tagDistance = relativePose.getTranslation().getNorm();
        xyStdDev = xyStdDevmodel.predict(tagDistance);
        thetaStdDev = thetaStdDevModel.predict(tagDistance);
        error = (xyStdWeight * xyStdDev) + (thetaStdWeight * thetaStdDev) + (ambiguityWeight * ambiguity);
        System.out.println("Ambiguity (reprojection error): " + ambiguity + ", Error (weighted): " + error);
        if (ambiguity < ambiguityThreshold) {
            historicalValues.put(error, positions);
        }
        //Remove values older than allowed
        for (Map.Entry<Double, Tuple<Pose3d, Double>> entry : historicalValues.entrySet()) {
            Double time = entry.getValue().getSecond();
            if (Timer.getFPGATimestamp() - time > maxHist) {
                historicalValues.remove(entry.getKey());
            }
        }
    }

    /**
     * @return a tuple containing a 3d position (pose3d) and a timestamp (double)
     */
    public Tuple<Pose3d, Double> getPoseSample() {
        return historicalValues.firstEntry().getValue();
    }

    /**
     * @param position the position of the values over the last second sorted by quality (error)
     * @return a tuple containing a 3d position (pose3d) and a timestamp (double)
     */
    public Tuple<Pose3d, Double> getPoseSample(int position) {
        var keys = historicalValues.keySet().toArray();
        var value = historicalValues.get(keys[position]);
        return value;
    }

    /**
     * Clears the poses recorded over the last second.
     * This should only be used in the event that the drivetrain
     * odometry drifts significantly over the course of a second.
     */
    public void clearPoses() {
        historicalValues.clear();
    }

}