package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import frc.robot.subsystems.vision.VisionHelpers.*;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class NorthStarInputs implements AprilTagInputs {
    String NT4Id;
    NetworkTable northStarNT4;
    DoubleArraySubscriber observationSubscriber;
    IntegerSubscriber fpsSubscriber;
    TimestampedDoubleArray[] queue;
    Map measurements = new HashMap<Double, AprilTagMeasurement>();

    public NorthStarInputs(String NT4Id) {
        this.NT4Id = NT4Id;
        northStarNT4 = NetworkTableInstance.getDefault().getTable(NT4Id);
        var outputTable = northStarNT4.getSubTable("output");
        observationSubscriber =
                outputTable
                        .getDoubleArrayTopic("observations")
                        .subscribe(
                                new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    }

    @Override
    public Map<Double, AprilTagMeasurement> getQueue() {
        return measurements;
    }

    @Override
    public void flushQueue() {
        measurements.clear();
    }

    @Override
    public void update() {
        queue = observationSubscriber.readQueue();
        double [] timestamps = new double[queue.length];
        double [][] frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            timestamps[i] = queue[i].timestamp / 1000000.0;
            frames[i] = queue[i].value;
        }
        long fps = fpsSubscriber.get();
        var
        }
    }
}
