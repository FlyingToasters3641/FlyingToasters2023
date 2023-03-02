package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;

public class VisionHelpers {
    public static class AprilTagConfig {
        public int id;
        public Pose3d pose;

        public AprilTagConfig(int id, Pose3d pose) {
            this.id = id;
            this.pose = pose;
        }
    }

    public static class AprilTagVisionIOInputs {
        public double[] timestamps = new double[]{};
        public double[][] frames = new double[][]{};
        public long fps = 0;
    }

    // Copyright (c) 2023 FRC 6328
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by an MIT-style
    // license that can be found in the LICENSE file at
    // the root directory of this project.

    public interface AprilTagVisionIO {
        public static class AprilTagVisionIOInputs {

            public double[] timestamps = new double[]{};
            public double[][] frames = new double[][]{};
            public long fps = 0;
        }

        public default void updateInputs(AprilTagVisionIOInputs inputs) {
        }
    }

}
