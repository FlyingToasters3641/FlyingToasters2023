package frc.robot.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.autonomous.RotationalTrajectory.*;

public class SwerveTrajectory {
    public Trajectory latTrajectory;
    public RotationalTrajectory rotTrajectory;

    public SwerveTrajectory(Trajectory generateTrajectory, RotationalTrajectory rotTrajectory) {
        latTrajectory = generateTrajectory;
        this.rotTrajectory = rotTrajectory;
    }

}
