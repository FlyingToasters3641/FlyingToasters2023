package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;

public class VisionHelpers {
    public static class AprilTagConfig {
        int id;
        Pose3d pose;

        public AprilTagConfig(int id, Pose3d pose) {
            this.id = id;
            this.pose = pose;
        }

        public int getId() {
            return id;
        }

        public Pose3d getPose() {
            return pose;
        }
    }

    /**
     * This is an extension of the wpilib AprilTag class that adds the
     * ability to place the active position of the april tag in the class
     * to pass it around
     */
    public static class TimeCorrectedAprilTag extends AprilTag{
        public Pose3d relativePose;
        public double timeStamp;
        public double ambiguity;
        NetworkTable table;

        public TimeCorrectedAprilTag(AprilTagConfig config, NetworkTable table) {
            super(config.id, config.pose);
            this.table = table;
        }

        public void updatePosition(Pose3d relativePose, double timeStamp, double ambiguity) {
            this.relativePose = relativePose;
            this.timeStamp = timeStamp;
            this.ambiguity = ambiguity;
        }
    
    }
}
