package frc.robot.subsystems.Vision.AprilTagSubsystem;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionHelpers.TimeCorrectedAprilTag;

public class AprilTagDetector extends SubsystemBase{
    NetworkTable detector;
    
    public AprilTagDetector(String nt4Id) {
        detector = NetworkTableInstance.getDefault().getTable(nt4Id).getSubTable("output");
    }
    
    TimeCorrectedAprilTag one = new TimeCorrectedAprilTag(0, null);
    
    
    public void getPosition() {
        
    }

}


