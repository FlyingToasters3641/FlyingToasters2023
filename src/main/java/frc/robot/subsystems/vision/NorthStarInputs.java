package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.TreeMap;

public class NorthStarInputs implements AprilTagInputs{
    String NT4Id;
    public NorthStarInputs(String NT4Id) {
        this.NT4Id = NT4Id;

    }
    @Override
    public TreeMap<Double, VisionHelpers.AprilTagMeasurement> getQueue() {
        return null;
    }

    @Override
    public void flushQueue() {

    }
}
