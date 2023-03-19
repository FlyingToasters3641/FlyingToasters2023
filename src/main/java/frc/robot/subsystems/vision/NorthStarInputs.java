package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.sql.Array;
import java.util.ArrayList;
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
        observationSubscriber = outputTable.getDoubleArrayTopic("observations").subscribe(new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
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
        //TODO: Check for null or length 0
        if (queue != null && queue.length > 0) {
            queue = observationSubscriber.readQueue();
            double[] timestamps = new double[queue.length];
            double[][] frames = new double[queue.length][];
            for (int i = 0; i < queue.length; i++) {
                timestamps[i] = queue[i].timestamp / 1000000.0;
                frames[i] = queue[i].value;
            }
            long fps = fpsSubscriber.get();

            for (int value = 0; value < queue.length; value++) {
                double timestamp = timestamps[value];
                double[] frame = frames[value];
                int id = (int) frame[NorthStarNetworkTables.NUMBER_OF_POSES.getValue()];
                double ambiguity = frame[NorthStarNetworkTables.ERROR_0.getValue()];
                var position =
                        new Pose3d(
                                frame[NorthStarNetworkTables.CAMERA_POSE_0_X_COMPONENT.getValue()],
                                frame[NorthStarNetworkTables.CAMERA_POSE_0_Y_COMPONENT.getValue()],
                                frame[NorthStarNetworkTables.CAMERA_POSE_0_Z_COMPONENT.getValue()],
                                new Rotation3d(
                                        new Quaternion(
                                                frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_W.getValue()],
                                                frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_X.getValue()],
                                                frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_Y.getValue()],
                                                frame[NorthStarNetworkTables.CAMERA_POSE_0_THETA_COMPONENT_Z.getValue()]
                                        )
                                )
                        );

                if ((int) frame[0] == 2 && frame[NorthStarNetworkTables.ERROR_1.getValue()] < ambiguity) {
                    ambiguity = frame[NorthStarNetworkTables.ERROR_1.getValue()];
                    position =
                            new Pose3d(
                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_X_COMPONENT.getValue()],
                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_Y_COMPONENT.getValue()],
                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_Z_COMPONENT.getValue()],
                                    new Rotation3d(
                                            new Quaternion(
                                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_THETA_COMPONENT_W.getValue()],
                                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_THETA_COMPONENT_X.getValue()],
                                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_THETA_COMPONENT_Y.getValue()],
                                                    frame[NorthStarNetworkTables.CAMERA_POSE_1_THETA_COMPONENT_Z.getValue()]
                                            )
                                    )
                            );
                }
                var measure = new AprilTagMeasurement(timestamp, id, position, ambiguity);
                measurements.put(timestamp, measure);

            }

        }
    }
}

