/*
 * Much of this was taken from or written to mimic the functions
 * of the Mechanical Advantage java classes AprilTagVisionIONorthstar
 * and AprilTagVision. I combined much of the functionality of the
 * aforementioned classes to allow for simpler testing and to make it
 * possible to plug in multiple different vision processors such as
 * PhotonVision. For reference, you can find the classes mentioned
 * here: https://github.com/Mechanical-Advantage/RobotCode2023/
 */

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
                                new double[]{},
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.sendAll(true)
                        );
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    }

    @Override
    public Map<Double, AprilTagMeasurement> getQueue() {
        update();
        var output = measurements;
        flushQueue();
        return output;
    }

    @Override
    public void flushQueue() {
        measurements.clear();
    }

    @Override
    public void update() {
        //Deserialize the output from the Northstar networktables entry
        if (queue != null && queue.length > 0) {
            queue = observationSubscriber.readQueue();
            double[] timestamps = new double[queue.length];
            double[][] frames = new double[queue.length][];
            for (int i = 0; i < queue.length; i++) {
                timestamps[i] = queue[i].timestamp / 1000000.0;
                frames[i] = queue[i].value;
            }
            long fps = fpsSubscriber.get();

            for (int tag = 0; tag < queue.length; tag++) {
                double timestamp = timestamps[tag];
                double[] frame = frames[tag];
                //If there's a duplicate tag pose, the tag id moves to position 17 in the frame (double array) and
                //position 9 becomes the error of the second tag pose.
                int id = frame[0] == 2 ? (int) frame[17] : (int) frame[9];
                double ambiguity = frame[NorthStarNetworkTables.ERROR_0.getValue()];
                var cameraPosition =
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
                //If there are multiple camera poses for the same tag in one frame, pick the best one.
                if (
                        (int) frame[0] == 2 &&
                                frame[NorthStarNetworkTables.ERROR_1.getValue()] < ambiguity
                ) {
                    ambiguity = frame[NorthStarNetworkTables.ERROR_1.getValue()];
                    cameraPosition =
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
                var measure = new AprilTagMeasurement(
                        timestamp,
                        id,
                        cameraPosition,
                        ambiguity,
                        fps
                );
                measurements.put(timestamp, measure);
            }
        }
    }
}
