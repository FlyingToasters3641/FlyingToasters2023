package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public interface AprilTagInputs {
    public Map<Double, AprilTagMeasurement> getQueue();
    //TODO: Determine if we really need flushQueue() as part of the interface
    public void flushQueue();

    public void update();

    public Pose3d getRelativeCameraPose();
}

