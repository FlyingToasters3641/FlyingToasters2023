package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;

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
}
