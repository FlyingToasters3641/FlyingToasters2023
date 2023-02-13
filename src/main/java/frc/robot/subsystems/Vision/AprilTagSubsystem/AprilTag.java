package frc.robot.subsystems.Vision.AprilTagSubsystem;

import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTag {
    Pose3d fieldPosition;
    Pose3d robotRelativePosition;
    int tagNumber;
    double timeStamp;

    public AprilTag(int tagNumber, Pose3d fieldPosition, Pose3d initialRobotRelativePosition, double initialTime) {
        this.tagNumber = tagNumber;
        this.fieldPosition = fieldPosition;
        robotRelativePosition = initialRobotRelativePosition;
        timeStamp = initialTime;
    }

    public void updatePosition(Pose3d robotRelativePosition, double timeStamp) {
        this.robotRelativePosition = robotRelativePosition;
        this.timeStamp = timeStamp;
    }

    public Pose3d getPosition() {
        return robotRelativePosition;
    }

    public double getTime() {
        return timeStamp;
    }

    public int getTagNumber() {
        return tagNumber;
    }

}
