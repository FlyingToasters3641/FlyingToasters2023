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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionHelpers.*;

import javax.sound.sampled.Line;
import java.util.HashMap;
import java.util.Map;

public class NorthStarInputs implements AprilTagInputs {

    String NT4Id;
    NetworkTable northStarNT4;
    DoubleArraySubscriber observationSubscriber;
    IntegerSubscriber fpsSubscriber;
    TimestampedDoubleArray[] queue;
    HashMap<Double, AprilTagMeasurement> measurements = new HashMap<>();
    private final Pose3d relativeCameraPosition;
    private Pose3d previousPosePosition;
    private double distanceTravelled;
    private double elapsedTime = 0.02;
    private LinearFilter highPassFilter = LinearFilter.highPass(0.1,0.02);
    private double prevTime = 0;
    private LinearFilter singlePoleFilter = LinearFilter.singlePoleIIR(0.05, elapsedTime);
    private LinearFilter movingAverageFilterX = LinearFilter.movingAverage(1);
    private LinearFilter movingAverageFilterY = LinearFilter.movingAverage(1);
    private MedianFilter medianAverageFilter = new MedianFilter(3);
    public NorthStarInputs(String NT4Id, Pose3d relativeCameraPosition) {
        this.NT4Id = NT4Id;
        this.relativeCameraPosition = relativeCameraPosition;
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
        HashMap<Double, AprilTagMeasurement> output = new HashMap<>(measurements);
        flushQueue();
        return output;
    }

    @Override
    public void flushQueue() {
        measurements.clear();
    }

    @Override
    public void update() {
        elapsedTime = Timer.getFPGATimestamp() - prevTime;
        queue = observationSubscriber.readQueue();
        //Deserialize the output from the Northstar networktables entry
        SmartDashboard.putNumber("Queue length", queue.length);
        if (queue != null && queue.length > 0) {
            double[] timestamps = new double[queue.length];
            double[][] frames = new double[queue.length][];
            for (int i = 0; i < queue.length; i++) {
                timestamps[i] = queue[i].timestamp / 1000000.0;
                frames[i] = queue[i].value;
            }
            long fps = fpsSubscriber.get();

            for (int tag = 0; tag < queue.length; tag++) {
                //TODO: Remove SmartDashboard calls, they eat up time on the robot loop
                SmartDashboard.putBoolean("NorthStarInputsCalled", true);
                double timestamp = timestamps[tag];
                double[] frame = frames[tag];
                //If there's a duplicate tag pose, the tag id moves to position 17 in the frame (double array) and
                //position 9 becomes the error of the second tag pose.
                //Another null check to make sure that there's actually a pose in the array
                if (frame[0] > 0.0) {
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
                        SmartDashboard.putBoolean("Unambiguous pose detected", false);
                    }
                    if (previousPosePosition != null && cameraPosition != null) {
                        distanceTravelled = cameraPosition.getTranslation().getDistance(previousPosePosition.getTranslation());
                }
                    double noise = highPassFilter.calculate(distanceTravelled);
                    if (ambiguity > 2 || noise > 0.4 || cameraPosition != null && cameraPosition.getX() >= 5.5 && cameraPosition.getX() <= 16.54175 - 5.5 /*|| distanceTravelled > 4.2672 / elapsedTime*/) {
                        SmartDashboard.putNumber(
                                "Number of poses thrown out:",
                                SmartDashboard.getNumber("Number of poses thrown out:", 0) + 1
                        );
                        SmartDashboard.putNumber("Noise", noise);
                        cameraPosition = null;
                        SmartDashboard.putNumber("ElapsedTime", Timer.getFPGATimestamp() - prevTime);
                        SmartDashboard.putBoolean("Unambiguous pose detected", false);
                        
                    }

                    if (cameraPosition != null) {
                        SmartDashboard.putBoolean("Unambiguous pose detected", true);
                        previousPosePosition = new Pose3d(cameraPosition.getTranslation(), cameraPosition.getRotation());
                        cameraPosition = new Pose3d(movingAverageFilterX.calculate(cameraPosition.getX()), movingAverageFilterY.calculate(cameraPosition.getY()), cameraPosition.getZ(), new Rotation3d(cameraPosition.getRotation().getQuaternion()));
                        
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
                        SmartDashboard.putBoolean("NorthStarInputsCalled", true);
                        SmartDashboard.putNumber("NorthStarX: ", cameraPosition.getX());
                        
                        measurements.put(ambiguity, measure);
                    }
                }
            }
        }
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public Pose3d getRelativeCameraPose() {
        return relativeCameraPosition;
    }
}
