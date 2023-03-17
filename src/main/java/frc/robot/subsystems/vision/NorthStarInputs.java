package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class NorthStarInputs implements AprilTagInputs {
    String NT4Id;
    NetworkTable northStarNT4;
    Map queue = new HashMap<Double, AprilTagMeasurement>();

    public NorthStarInputs(String NT4Id) {
        this.NT4Id = NT4Id;
        northStarNT4 = NetworkTableInstance.getDefault().getTable(NT4Id).getSubTable("output");
    }

    @Override
    public Map<Double, AprilTagMeasurement> getQueue() {
        return queue;
    }

    @Override
    public void flushQueue() {
        queue.clear();
    }

    @Override
    public void update() {

    }
}
