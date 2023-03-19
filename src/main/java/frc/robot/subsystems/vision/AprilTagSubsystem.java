package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

public class AprilTagSubsystem extends SubsystemBase {
    AprilTagInputs[] detectors;
    public AprilTagSubsystem (AprilTagInputs... detectors) {
        this.detectors = detectors;
    }



    @Override
    public void periodic() {

    }
}
