package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.HashMap;
import java.util.Map;

public class NorthStarInputs implements AprilTagInputs {
    String NT4Id;
    NetworkTable northStarNT4;
    DoubleArraySubscriber observationSubscriber;
    IntegerSubscriber fpsSubscriber;
    TimestampedDoubleArray[] queue;
    HashMap<Double, AprilTagMeasurement> measurements = new HashMap<Double, AprilTagMeasurement>();

    public NorthStarInputs(String NT4Id) {
        this.NT4Id = NT4Id;
        northStarNT4 = NetworkTableInstance.getDefault().getTable(NT4Id);
        var outputTable = northStarNT4.getSubTable("output");
        observationSubscriber =
                outputTable
                        .getDoubleArrayTopic("observations")
                        .subscribe(
                                new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    }

    @Override
    public Map<Double, AprilTagMeasurement> getQueue() {
        return measurements;
    }

    @Override
    public void flushQueue() {
        measurements.clear();
    }

    @Override
    public void update() {
        //TODO: Check for null or length < 0
        queue = observationSubscriber.readQueue();
        double[] timestamps = new double[queue.length];
        double[][] frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            timestamps[i] = queue[i].timestamp / 1000000.0;
            frames[i] = queue[i].value;
        }
        long fps = fpsSubscriber.get();
        //TODO: Refactor to inline
        for (int value = 0; value < queue.length; value++) {
            double timestamp = timestamps[value];
            double[] frame = frames[value];
            int id = (int) frame[NorthStarNetworkTables.NUMBER_OF_POSES.getValue()];
            double x = frame[NorthStarNetworkTables.CAMERA_POSE_0_X_COMPONENT.getValue()];
            double y = frame[NorthStarNetworkTables.CAMERA_POSE_0_Y_COMPONENT.getValue()];
            double z = frame[NorthStarNetworkTables.CAMERA_POSE_0_Z_COMPONENT.getValue()];
            double wTheta = frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_W.getValue()];
            double xTheta = frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_X.getValue()];
            double yTheta = frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_Y.getValue()];
            double zTheta = frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_Z.getValue()];
            double ambiguity = frame[NorthStarNetworkTables.ERROR_0.getValue()];
            var rotation = new Rotation3d(new Quaternion(wTheta, xTheta, yTheta, zTheta));
            var position = new Pose3d(x, y, z, rotation);
            var measure = new AprilTagMeasurement(timestamp, id, position, ambiguity);
            measurements.put(timestamp, measure);
        }
    }
}

