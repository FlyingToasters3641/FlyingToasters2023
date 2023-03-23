package frc.robot.subsystems.vision;

import com.sun.source.tree.Tree;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.*;

import static frc.robot.Constants.tagConfigs;


public class AprilTagSubsystem extends SubsystemBase {
    AprilTagInputs[] detectors;
    double totalDistance = 0.0;
    private static final double xyStdDevCoefficient = 0.01;
    private static final double thetaStdDevCoefficient = 0.01;

    TreeMap<Double, AprilTagMeasurement> robotPoses = new TreeMap<Double, AprilTagMeasurement>();
    TreeMap<Integer, AprilTagMeasurement> tagPoses = new TreeMap<Integer, AprilTagMeasurement>();

    public AprilTagSubsystem(AprilTagInputs... detectors) {
        this.detectors = detectors;
    }

    private void updatePoses() {
        for (AprilTagInputs detector : detectors) {
            var detectorQueue = detector.getQueue();
            robotPoses.putAll(detectorQueue);
        }
        //TODO: Replace length with tag configuration length
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

    }

    @Override
    public void periodic() {
        updatePoses();
    }
}
