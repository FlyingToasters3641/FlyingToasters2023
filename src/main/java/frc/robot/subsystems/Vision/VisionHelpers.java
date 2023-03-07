package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
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

    private static Pose3d openCVPoseToWPILibPose(Vector<N3> tvec, Vector<N3> rvec) {
        return new Pose3d(
                new Translation3d(tvec.get(2, 0), -tvec.get(0, 0), -tvec.get(1, 0)),
                new Rotation3d(
                        VecBuilder.fill(rvec.get(2, 0), -rvec.get(0, 0), -rvec.get(1, 0)),
                        Math.sqrt(
                                Math.pow(rvec.get(0, 0), 2)
                                        + Math.pow(rvec.get(1, 0), 2)
                                        + Math.pow(rvec.get(2, 0), 2))));
    }

    public static class Observation {
        public static Double timeStamp;
        public static Double error;
        public static Pose3d pose;

        public Observation(double error, Double timeStamp, Pose3d pose) {
            this.error = error;
            this.timeStamp = timeStamp;
            this.pose = pose;
        }
    }

}
