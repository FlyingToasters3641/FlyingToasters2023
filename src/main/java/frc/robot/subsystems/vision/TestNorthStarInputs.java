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
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import java.util.*;

public class TestNorthStarInputs implements AprilTagInputs {

    double[][] queue = new double[800][];
    HashMap<Double, AprilTagMeasurement> measurements = new HashMap<>();
    private final Pose3d relativeCameraPosition;
    double[][] NorthStarData;
    double[] allTimestamps;
    double[] timestampQueue = new double[800];
    Double NorthStarInitialTimestamp;
    private double curTime;
    private double requestTime;
    private double prevRequestTime;

    public TestNorthStarInputs(Pose3d relativeCameraPosition, String csvName) {
        this.relativeCameraPosition = relativeCameraPosition;

        Path openFile = Paths.get(csvName);

        try {
            var output = Files.readAllLines(openFile);
            NorthStarData = new double[output.size()][];
            allTimestamps = new double[output.size()];
//            Arrays.fill(NorthStarData, output.size());
            for (int i = 0; i < output.size(); i++) {
                int b = i;

                String line = output.get(i);
                String[] frameString = line.split(",");
                if (!(Double.parseDouble(frameString[0]) > 0)) {
                    b -= 1;
                }
                if (Double.parseDouble(frameString[0]) > 0) {
                    allTimestamps[b] = Double.parseDouble(frameString[0]);
                }
                double[] frame = new double[frameString.length - 1];
                for (int j = 1; j < frameString.length; j++) {
                    String stringValue = frameString[j];
                    if (Double.parseDouble(frameString[0]) > 0) {
                        frame[j - 1] = Double.parseDouble(stringValue);
                    }
                }
                if (Double.parseDouble(frameString[0]) > 0) {
                NorthStarData[i] = frame;}
                if (NorthStarInitialTimestamp == null && Double.parseDouble(frameString[0]) > 0.000) {
                    NorthStarInitialTimestamp = Double.parseDouble(frameString[0]);}
                else {NorthStarInitialTimestamp = NorthStarInitialTimestamp;}

            }
        } catch (IOException e) {
            System.out.println("Unable to open test output (ignore if this ends up in the competition code)");
            System.out.println(Arrays.toString(e.getStackTrace()));
        }

        curTime = NorthStarInitialTimestamp;
        prevRequestTime = 0;

    }

    @Override
    public Map<Double, AprilTagMeasurement> getQueue() {
        curTime += 0.02 * 1000000.0;
        requestTime = curTime;
        update();
        var output = measurements;
        flushQueue();
        Arrays.fill(timestampQueue, 0.0);
        prevRequestTime = requestTime;

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
            //TODO: queue will always be null in non-test version
            int z = 0;
            for (int i = 0; i < allTimestamps.length; i++) {
                double value = allTimestamps[i];
                if (value <= requestTime && value >= prevRequestTime && value > 0.00000) {
                    queue[z] = NorthStarData[i];
                    timestampQueue[z] = allTimestamps[i];
                    z += 1;
                }
            }

            double[] timestamps = new double[queue.length];
            double[][] frames = new double[queue.length][];
            for (int i = 0; i < queue.length; i++) {
                timestamps[i] = timestampQueue[i] / 1000000.0;
                frames[i] = queue[i];
            }
            long fps = 50;

            for (int tag = 0; tag < queue.length; tag++) {
                if (queue[tag] != null) {
                    double timestamp = timestamps[tag];
                    double[] frame = frames[tag];
                    //If there's a duplicate tag pose, the tag id moves to position 17 in the frame (double array) and
                    //position 9 becomes the error of the second tag pose.
                    int id = (int) frame[0] == 2 ? (int) frame[17] : (int) frame[9];
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
                                    frame[NorthStarNetworkTables.ERROR_1.getValue()] < ambiguity * 0.15
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
                    } else if (
                            (int) frame[0] == 2 &&
                                    ambiguity > frame[NorthStarNetworkTables.ERROR_1.getValue()] * 0.15
                    ) {
                        cameraPosition = null;
                    }
                    if (cameraPosition != null) {
                        var measure = new AprilTagMeasurement(
                                timestamp,
                                id,
                                cameraPosition.transformBy(
                                        new Transform3d(
                                                relativeCameraPosition.getTranslation(),
                                                relativeCameraPosition.getRotation())
                                                .inverse()
                                ),
                                cameraPosition,
                                ambiguity,
                                fps
                        );
                        measurements.put(timestamp, measure);
                    }
                }
            }
        }
    }

    @Override
    public Pose3d getRelativeCameraPose() {
        return relativeCameraPosition;
    }
}