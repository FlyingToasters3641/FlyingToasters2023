package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.*;
import java.util.function.Consumer;

import static frc.robot.Constants.tagConfigs;


public class AprilTagSubsystem extends SubsystemBase {
    AprilTagInputs[] detectors;
    double totalDistance = 0.0;
    private static final double xyStdDevCoefficient = 0.01;
    private static final double thetaStdDevCoefficient = 0.01;
    private TreeMap<Integer, AprilTagMeasurement> prevTagPoses = new TreeMap<>();

    private TreeMap<Double, AprilTagMeasurement> robotPoses = new TreeMap<>();
    private TreeMap<Integer, AprilTagMeasurement> tagPoses = new TreeMap<>();
    private final Consumer<AprilTagMeasurement> poseEstimator;
    private Pose3d outputPose;

    public AprilTagSubsystem(Consumer<AprilTagMeasurement> poseEstimator, AprilTagInputs... detectors) {
        this.poseEstimator = poseEstimator;
        this.detectors = detectors;
    }

    private void updatePoses() {
        for (AprilTagInputs detector : detectors) {
            var detectorQueue = detector.getQueue();
            robotPoses.putAll(detectorQueue);
        }
        for (int i = 1; i <= tagConfigs.length; i++) {
            for (Map.Entry<Double, AprilTagMeasurement> m : robotPoses.entrySet()) {
                if (m.getValue().ID == i) {
                    tagPoses.put(m.getValue().ID, m.getValue());
                    SmartDashboard.putNumber("Tag " + m.getValue().ID + " X position:", m.getValue().pose.getX());
                    SmartDashboard.putNumber("Tag " + m.getValue().ID + " Y position:", m.getValue().pose.getY());
                    SmartDashboard.putNumber("Tag " + m.getValue().ID + " Z position:", m.getValue().pose.getZ());
                    totalDistance += tagConfigs[i - 1].pose.getTranslation().getDistance(m.getValue().nonTransformedPose.getTranslation());
                }
            }

        }
        double avgDistance = totalDistance / tagPoses.size();
        double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();

        for (Map.Entry<Integer, AprilTagMeasurement> entry : tagPoses.entrySet() ) {
            int id = entry.getValue().getID();
            double currentTimeStamp = entry.getValue().getTimestamp();

            if (prevTagPoses != null && prevTagPoses.get(id).getTimestamp() < currentTimeStamp) {
                poseEstimator.accept(entry.getValue());
            } else if (prevTagPoses == null) {
                poseEstimator.accept(entry.getValue());
            }
        }

        prevTagPoses = tagPoses;
    }

    @Override
    public void periodic() {
        updatePoses();
    }
}
