package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.TreeMap;

public class NorthStarInputs implements AprilTagInputs{
    String NT4Id;
    NetworkTable northStarNT4;
    public NorthStarInputs(String NT4Id) {
        this.NT4Id = NT4Id;
        northStarNT4 = NetworkTableInstance.getDefault().getTable(NT4Id).getSubTable("output");
    }
    @Override
    public TreeMap<Double, VisionHelpers.AprilTagMeasurement> getQueue() {
        return null;
    }

    @Override
    public void flushQueue() {

    }

    @Override
    public void update() {

    }
}
