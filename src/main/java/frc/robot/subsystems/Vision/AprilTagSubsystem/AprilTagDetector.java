package frc.robot.subsystems.Vision.AprilTagSubsystem;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionHelpers.AprilTagConfig;
import frc.robot.subsystems.Vision.VisionHelpers.TimeCorrectedAprilTag;
/*the working plan is for the TimeCorrectedAprilTag (name should be changed) class to handle all of the filtering and updating on a per april tag basis,
* this simplifies it as it means that I don't have to work with filtering multidimensional arrays which would be an absolute nightmare
* I have to work out a system for how to take in multiple values from multiple cameras, I'm thinking of just selecting the newest value if there's a conflict
* an update method will then be called by the AprilTagDetector in the periodic method.
* I still need to figure out how the networktables values are laid out and I need to figure out how to send an instance of networktables into the TimeCorrectedAprilTag
* class or if I should just skip dependency injection and declare it in the TimeCorrectedAprilTag class.
* I also need to figure out what the field positions of the april tags are
*/
//TODO: Need to move TimeCorrectedAprilTag to it's own file and rename it, this needs it
public class AprilTagDetector extends SubsystemBase{
    NetworkTable detector;
    AprilTagConfig config;
    
    public AprilTagDetector(String nt4Id) {
        detector = NetworkTableInstance.getDefault().getTable(nt4Id).getSubTable("output");
    }
    
    

}


