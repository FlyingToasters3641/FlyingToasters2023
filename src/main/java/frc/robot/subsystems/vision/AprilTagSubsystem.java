package frc.robot.subsystems.vision;


import com.sun.source.tree.Tree;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static frc.robot.Constants.tagConfigs;


public class AprilTagSubsystem extends SubsystemBase {
    AprilTagInputs[] detectors;
    double totalDistance = 0.0;

    private static final double xyStdDevCoefficient = 0.01;
    private static final double thetaStdDevCoefficient = 0.01;
    private double prevTagPoseTime = 0;
    private Pose2d prevTagPose;
    private double xDist = 0;
    private double yDist = 0;
    private double xDiff = 0;
    private double yDiff = 0;
    private TreeMap<Double, Pose2d> historicalDrivetrainPoses = new TreeMap<Double, Pose2d>(Collections.reverseOrder());
    private TreeMap<Double, Pose2d> historicalAprilTagPoses = new TreeMap<Double, Pose2d>(Collections.reverseOrder());
    private TreeMap<Integer, AprilTagMeasurement> prevTagPoses;
    //NOTE: This is in reverse order, meaning that the worst poses will be first in the map,
    //due to the way that we're iterating over it, it makes sense to do that because it means
    //that the last poses that are written to the tagPoses treemap are the best ones
    private final TreeMap<Double, AprilTagMeasurement> robotPoses = new TreeMap<>(Collections.reverseOrder());
    private final TreeMap<Integer, AprilTagMeasurement> tagPoses = new TreeMap<>();
    private final Consumer<AprilTagMeasurement> poseEstimator;
    private final Supplier<Pose2d> getDriveOdometryPose;
    int testRemoveMe = 0;
    //ShuffleboardTab tab = Shuffleboard.getTab("NorthStar");


    public AprilTagSubsystem(Consumer<AprilTagMeasurement> poseEstimator, Supplier<Pose2d> getDriveOdometryPose, AprilTagInputs... detectors) {
        this.poseEstimator = poseEstimator;
        this.detectors = detectors;
        this.getDriveOdometryPose = getDriveOdometryPose;

    }

    private void updatePoses() {
        for (AprilTagInputs detector : detectors) {
            var detectorQueue = detector.getQueue();
            /*TODO: This is better (the double is now ambiguity), but we should add something to validate that the pose is fairly recent,
               from initial tests, the subsystem keeps the values fairly fresh, but just to double check, we should also double check the treemap
               reversal.*/
            robotPoses.putAll(detectorQueue);
            testRemoveMe += robotPoses.size();
            SmartDashboard.putNumber("robotPoses number", testRemoveMe);
            SmartDashboard.putBoolean("AprilTagSubsystemCalled", true);
        }
        for (int i = 1; i <= tagConfigs.length; i++) {
            for (Map.Entry<Double, AprilTagMeasurement> m : robotPoses.entrySet()) {
                if (m.getValue().ID == i) {
                    tagPoses.put(m.getValue().ID, m.getValue());
                    //    var positionForTest = new Field2d();

                    //  positionForTest.setRobotPose(m.getValue().getPose().toPose2d());

                    totalDistance += tagConfigs[i - 1].pose.getTranslation().getDistance(m.getValue().nonTransformedPose.getTranslation());
                }
            }

        }
        double avgDistance = totalDistance / tagPoses.size();
        double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();

        for (Map.Entry<Integer, AprilTagMeasurement> entry : tagPoses.entrySet()) {

            int id = entry.getValue().getID();
            if (prevTagPose != null && historicalDrivetrainPoses != null && historicalDrivetrainPoses.size() != 0) {
                xDist = entry.getValue().getPose().getX() - prevTagPose.getX();
                yDist = entry.getValue().getPose().getY() - prevTagPose.getY();
                Pose2d prevTagPoseDrivetrainMatch = new Pose2d();
                Pose2d curTagPoseDrivetrainMatch = new Pose2d();
                double prevTimestamp = 999999999;
                double prevTimeDiff = 999999999;
                double curTimestamp = 999999999;
                double curTimeDiff = 999999999;
                for (Map.Entry<Double, Pose2d> pose2dEntry : historicalDrivetrainPoses.entrySet()) {
                    if (Math.abs(pose2dEntry.getKey() - entry.getValue().getTimestamp()) < curTimeDiff) {
                        curTimeDiff = Math.abs(pose2dEntry.getKey() - entry.getValue().getTimestamp());
                        curTimestamp = pose2dEntry.getKey();
                    }
                    if (Math.abs(pose2dEntry.getKey() - entry.getValue().getTimestamp()) < prevTimeDiff) {
                        prevTimeDiff = Math.abs(pose2dEntry.getKey() - entry.getValue().getTimestamp());
                        prevTimestamp = pose2dEntry.getKey();
                    }
                }
                prevTagPoseDrivetrainMatch = historicalDrivetrainPoses.get(prevTimestamp);
                curTagPoseDrivetrainMatch = historicalDrivetrainPoses.get(curTimestamp);

                prevTagPoseDrivetrainMatch = prevTagPoseDrivetrainMatch.interpolate(curTagPoseDrivetrainMatch, (prevTimestamp - prevTagPoseTime) / (curTimestamp - prevTagPoseTime));
                xDiff = Math.abs(Math.abs(curTagPoseDrivetrainMatch.getX() - prevTagPoseDrivetrainMatch.getX()) - Math.abs(xDist));
                yDiff = Math.abs(Math.abs(curTagPoseDrivetrainMatch.getY() - prevTagPoseDrivetrainMatch.getY()) - Math.abs(yDist));
                xDiff *= 8;
                yDiff *= 8;
                SmartDashboard.putNumber("xStdDev", xDiff);
                SmartDashboard.putNumber("yStdDev", yDiff);
                entry.getValue().addStdDevs(xDiff, yDiff);
            }


            SmartDashboard.putNumber("Tag detected:", id);
            double currentTimeStamp = entry.getValue().getTimestamp();
            // SmartDashboard.putBoolean("Got to consumer caller", true);
            if (prevTagPoses != null && prevTagPoses.get(id) != null && prevTagPoses.size() > 0 && prevTagPoses.get(id).getTimestamp() < currentTimeStamp) {
                //  SmartDashboard.putBoolean("Consumer Called:", true);
                SmartDashboard.putNumber("Ambiguity", entry.getValue().getAmbiguity());
                historicalAprilTagPoses.put(entry.getValue().timestamp, entry.getValue().getPose().toPose2d());
                prevTagPose = entry.getValue().getPose().toPose2d();
                prevTagPoseTime = entry.getValue().getTimestamp();
                poseEstimator.accept(entry.getValue());
            } else if (prevTagPoses == null || prevTagPoses.size() == 0 || prevTagPoses.get(id) == null) {
                //  SmartDashboard.putBoolean("Consumer Called:", true);
                SmartDashboard.putNumber("Ambiguity", entry.getValue().getAmbiguity());
                historicalAprilTagPoses.put(entry.getValue().timestamp, entry.getValue().getPose().toPose2d());
                prevTagPose = entry.getValue().getPose().toPose2d();
                prevTagPoseTime = entry.getValue().getTimestamp();
                poseEstimator.accept(entry.getValue());
            } else {
                // SmartDashboard.putBoolean("Consumer Called:", false);
            }
        }

        prevTagPoses = new TreeMap<>(tagPoses);
        robotPoses.clear();
        tagPoses.clear();
    }

    @Override
    public void periodic() {
        //This is awful, but I don't care about this code anymore
        historicalDrivetrainPoses.put(Timer.getFPGATimestamp(), getDriveOdometryPose.get());
        ArrayList<Double> valuesToRemove = new ArrayList<>();

        Double time = Timer.getFPGATimestamp();

        cleanupMap(valuesToRemove, time, historicalDrivetrainPoses);
        cleanupMap(valuesToRemove, time, historicalAprilTagPoses);

        updatePoses();
    }

    private void cleanupMap(ArrayList<Double> valuesToRemove, Double time, TreeMap<Double, Pose2d> historicalDrivetrainPoses) {
        for (Map.Entry<Double, Pose2d> entry : historicalDrivetrainPoses.entrySet()) {
            if (time - entry.getKey() > 1) {
                valuesToRemove.add(entry.getKey());
            }
        }
        for (Double val : valuesToRemove) {
            historicalDrivetrainPoses.remove(val);
        }
        valuesToRemove.clear();
    }
}
