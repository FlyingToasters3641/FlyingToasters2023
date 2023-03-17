package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.HashMap;
import java.util.TreeMap;

public interface AprilTagInputs {
    public TreeMap<Double, AprilTagMeasurement> getQueue();
    public void flushQueue();
}

