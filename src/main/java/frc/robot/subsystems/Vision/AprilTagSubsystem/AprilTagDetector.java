package frc.robot.subsystems.Vision.AprilTagSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagDetector extends SubsystemBase{
    NetworkTable detector;
    
    public AprilTagDetector(String nt4Id) {
        detector = NetworkTableInstance.getDefault().getTable(nt4Id).getSubTable("output");
    }
    AprilTag one = new AprilTag(0, null, null, 0);
    public void getPosition() {
        
    }

}


