package frc.robot.subsystems.Vision.AprilTagSubsystem;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.subsystems.Vision.VisionHelpers.AprilTagConfig;

/**
     * This is an extension of the wpilib AprilTag class that adds the
     * ability to place the active position of the april tag in the class
     * to pass it around
     */
    public class TimeCorrectedAprilTag extends AprilTag{
        public Pose3d relativePose;
        public double timeStamp;
        public double ambiguity;
        NetworkTable table;

        public TimeCorrectedAprilTag(AprilTagConfig config, NetworkTable table) {
            super(config.getId(), config.getPose());
            this.table = table;
        }

        public void updatePosition(Pose3d relativePose, double timeStamp, double ambiguity) {
            this.relativePose = relativePose;
            this.timeStamp = timeStamp;
            this.ambiguity = ambiguity;
        }
    
    }