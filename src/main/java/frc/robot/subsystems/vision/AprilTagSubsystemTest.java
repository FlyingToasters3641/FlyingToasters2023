package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.*;

class AprilTagSubsystemTest {
    AprilTagSubsystem aprilTagSubsystem;
    Consumer<VisionHelpers.AprilTagMeasurement> printMeasurement = measurement -> System.out.println(measurement);
    AprilTagInputs input = new TestNorthStarInputs(new Pose3d(), "src/main/java/frc/robot/subsystems/vision/output.csv");
    @BeforeEach
    void Setup() {
        aprilTagSubsystem = new AprilTagSubsystem(printMeasurement, input);
    }

    @Test
    void StartTest() {
        while(true) {
        aprilTagSubsystem.periodic();
        }
    }

}