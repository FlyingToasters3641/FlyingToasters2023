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

    public static class Tuple<A, B> {
        private final A first;
        private final B second;
    
        public Tuple(A first, B second) {
            this.first = first;
            this.second = second;
        }
    
        public A getFirst() {
            return first;
        }
    
        public B getSecond() {
            return second;
        }
    }
    
}
