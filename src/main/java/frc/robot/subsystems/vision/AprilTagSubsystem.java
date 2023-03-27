package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.*;
import java.util.function.Consumer;

import static frc.robot.Constants.tagConfigs;


public class AprilTagSubsystem extends SubsystemBase{
    AprilTagInputs[] detectors;
    double totalDistance = 0.0;

    private static final double xyStdDevCoefficient = 0.01;
    private static final double thetaStdDevCoefficient = 0.01;
    private TreeMap<Integer, AprilTagMeasurement> prevTagPoses;

    private TreeMap<Double, AprilTagMeasurement> robotPoses = new TreeMap<>();
    private TreeMap<Integer, AprilTagMeasurement> tagPoses = new TreeMap<>();
    private final Consumer<AprilTagMeasurement> poseEstimator;
    int testRemoveMe = 0;
    private Pose3d outputPose;
    //ShuffleboardTab tab = Shuffleboard.getTab("NorthStar");



    public AprilTagSubsystem(Consumer<AprilTagMeasurement> poseEstimator, AprilTagInputs... detectors) {
        this.poseEstimator = poseEstimator;
        this.detectors = detectors;

    }

    private void updatePoses() {
        for (AprilTagInputs detector : detectors) {
            var detectorQueue = detector.getQueue();
            //TODO: This may overwrite the previous poses, try to add something that will get the least ambiguous poses
            robotPoses.putAll(detectorQueue);
            testRemoveMe += robotPoses.size();
            SmartDashboard.putNumber("robotPoses number", testRemoveMe);
            SmartDashboard.putBoolean("AprilTagSubsystemCalled", true);
        }
        for (int i = 1; i <= tagConfigs.length; i++) {
            for (Map.Entry<Double, AprilTagMeasurement> m : robotPoses.entrySet()) {
                if (m.getValue().ID == i) {
                    tagPoses.put(m.getValue().ID, m.getValue());
                    var positionForTest = new Field2d();

                    positionForTest.setRobotPose(m.getValue().getPose().toPose2d());

                    totalDistance += tagConfigs[i - 1].pose.getTranslation().getDistance(m.getValue().nonTransformedPose.getTranslation());
                }
            }

        }
        double avgDistance = totalDistance / tagPoses.size();
        double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();

        for (Map.Entry<Integer, AprilTagMeasurement> entry : tagPoses.entrySet() ) {
            int id = entry.getValue().getID();
            SmartDashboard.putNumber("Tag detected:", id);
            double currentTimeStamp = entry.getValue().getTimestamp();
            SmartDashboard.putBoolean("Got to consumer caller", true);
            if (prevTagPoses != null && prevTagPoses.get(id) != null && prevTagPoses.size() > 0 && prevTagPoses.get(id).getTimestamp() < currentTimeStamp) {
                SmartDashboard.putBoolean("Consumer Called:", true);
                poseEstimator.accept(entry.getValue());
            } else if (prevTagPoses == null || prevTagPoses.size() == 0 || prevTagPoses.get(id) == null) {
                SmartDashboard.putBoolean("Consumer Called:", true);
                poseEstimator.accept(entry.getValue());
            } else {SmartDashboard.putBoolean("Consumer Called:", false);}
        }

        prevTagPoses = new TreeMap<Integer, AprilTagMeasurement>(tagPoses);
    }

    @Override
    public void periodic() {
        updatePoses();
    }
}
