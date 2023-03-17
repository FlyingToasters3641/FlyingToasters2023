package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionHelpers {
    public static class AprilTagMeasurement extends AprilTag {
        double Timestamp;
        double ambiguity;

        public AprilTagMeasurement(int ID, double Timestamp, Pose3d pose, double ambiguity) {
            super(ID, pose);
            this.Timestamp = Timestamp;
            this.ambiguity = ambiguity;
        }
    }
}
