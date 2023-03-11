package frc.robot.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.Arrays;

public class TrajectoryHelpers {

  public enum Speeds {
    Safe(0.5, 0.5),
    Slow(1.0, 1.5),
    MedSlow(2.0, 2.5),
    Med(3.0, 2.5),
    Fast(4.0, 3.0),
    Ludicrous(5.0, 9.0); // Actually faster than the robot can go, so make sure to use voltage

    // constraints!

    public double velocity;
    public double acceleration;
    public PathConstraints Constraints;

    private Speeds(double velocity, double acceleration) {
      this.velocity = velocity;
      this.acceleration = acceleration;
      this.Constraints = new PathConstraints(velocity, acceleration);
    }
  }

  // Create config for trajectory
  public static TrajectoryConfig trajectoryConfig(Speeds speed) {
    return trajectoryConfig(speed, false);
  }

  public static TrajectoryConfig trajectoryConfig(
    Speeds speed,
    boolean backwards
  ) {
    TrajectoryConfig config;
    if (Constants.Trajectories.safetyMode) {
      config =
        new TrajectoryConfig(Speeds.Safe.velocity, Speeds.Safe.acceleration);
    } else {
      config = new TrajectoryConfig(speed.velocity, speed.acceleration);
    }

    config.setReversed(backwards);
    config.setKinematics(Constants.DrivetrainConstants.KINEMATICS); // Add kinematics to ensure max speed is actually
    // obeyed

    return config;
  }

  public static HolonomicDriveCommand FollowTrajectory(
    DrivetrainSubsystem drive,
    PoseEstimatorSubsystem poseEstimator,
    PathPlannerTrajectory swerveTrajectory
  ) {
    PathPlannerTrajectory trajectory = swerveTrajectory;
    return new HolonomicDriveCommand(
      trajectory,
      () -> {
        Pose2d pose = poseEstimator.getCurrentPose();
        if (pose == null) {
          pose = new Pose2d();
        }
        return pose;
      },
      new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(
          1,
          0,
          0,
          Constants.AutoConstants.kThetaControllerConstraints
        )
      ),
      Constants.DrivetrainConstants.KINEMATICS,
      (p0, p1, p2) -> drive.setVelocities(p0, p1, p2)
    );
  }

  public static Pose2d waypoint(double x, double y, double degrees) {
    return new Pose2d(x, y, new Rotation2d(degrees * Math.PI / 180.0));
  }

  public static Transform2d transform(double x, double y, double degrees) {
    return new Transform2d(
      new Translation2d(x, y),
      Rotation2d.fromDegrees(degrees)
    );
  }

  public static Translation2d point(double x, double y) {
    return new Translation2d(x, y);
  }

  public static Trajectory trajectory(
    TrajectoryConfig config,
    Pose2d... waypoints
  ) {
    return TrajectoryGenerator.generateTrajectory(
      Arrays.asList(waypoints),
      config
    );
  }

  public static Rotation2d getStraightPathAngle(
    Pose2d start,
    Pose2d end,
    boolean isReversed
  ) {
    double sx = start.getX();
    double sy = start.getY();
    double ex = end.getX();
    double ey = end.getY();
    double deltaY = ey - sy;
    double deltaX = ex - sx;
    double rotationRadians = Math.atan((ey - sy) / (ex - sx));
    if (deltaX < 0) {
      rotationRadians = rotationRadians + Math.PI;
    } else if (deltaY < 0 && deltaX >= 0) {
      rotationRadians = rotationRadians + 2 * Math.PI;
    }
    if (isReversed == true && rotationRadians >= Math.PI) {
      rotationRadians = rotationRadians - Math.PI;
    } else if (isReversed == true && rotationRadians <= Math.PI) {
      rotationRadians = rotationRadians + Math.PI;
    }
    return new Rotation2d(rotationRadians);
  }
}
