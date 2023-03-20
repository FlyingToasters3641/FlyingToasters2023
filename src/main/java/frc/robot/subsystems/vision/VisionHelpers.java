package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionHelpers {
    public static class AprilTagMeasurement {
        double timestamp;
        double ambiguity;
        int ID;
        long fps;
        Pose3d pose;

        public AprilTagMeasurement(double timestamp, int ID, Pose3d pose, double ambiguity, long fps) {
            this.ID = ID;
            this.pose = pose;
            this.timestamp = timestamp;
            this.ambiguity = ambiguity;
            this.fps = fps;
        }
    }
    //NOTE: You must place the theta components in a quaternion
    public enum NorthStarNetworkTables {
        NUMBER_OF_POSES(0),
        ERROR_0(1),
        CAMERA_POSE_0_X_COMPONENT(2),
        CAMERA_POSE_0_Y_COMPONENT(3),
        CAMERA_POSE_0_Z_COMPONENT(4),
        CAMERA_POSE_0_THETA_COMPONENT_W(5),
        CAMERA_POSE_0_THETA_COMPONENT_X(6),
        CAMERA_POSE_0_THETA_COMPONENT_Y(7),
        CAMERA_POSE_0_THETA_COMPONENT_Z(8),
        ERROR_1(9),
        CAMERA_POSE_1_X_COMPONENT(10),
        CAMERA_POSE_1_Y_COMPONENT(11),
        CAMERA_POSE_1_Z_COMPONENT(12),
        CAMERA_POSE_1_THETA_COMPONENT_W(13),
        CAMERA_POSE_1_THETA_COMPONENT_X(14),
        CAMERA_POSE_1_THETA_COMPONENT_Y(15),
        CAMERA_POSE_1_THETA_COMPONENT_Z(16);

        private final int i;

        NorthStarNetworkTables(int i) {
            this.i = i;
        }

        int getValue() {
            return i;
        }
    }
}
