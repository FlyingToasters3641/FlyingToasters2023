package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;

import static org.junit.jupiter.api.Assertions.*;

class AprilTagSubsystemTest {
    @BeforeEach
        // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    @Test
    void testTags() {
        var detectTest = new NorthStarInputs("NorthStar", new Pose3d(0,0,0, new Rotation3d(0,0,0)));
        new AprilTagSubsystem(detectTest);
    }

}