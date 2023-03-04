package frc.robot.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveTrajectory {
    public Trajectory latTrajectory;
    public RotationalTrajectory rotTrajectory;

    public SwerveTrajectory(Trajectory generateTrajectory, RotationalTrajectory rotTrajectory) {
        latTrajectory = generateTrajectory;
        this.rotTrajectory = rotTrajectory;
    }

}
