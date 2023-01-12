package frc.robot.autonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.HolonomicDriveController;
import java.io.IOException;
import frc.robot.Constants.*;
import frc.robot.lib.Units.*;

import java.nio.file.Path;
import java.util.Arrays;

import org.opencv.core.Mat;

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

        private Speeds(double velocity, double acceleration) {
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    // Create config for trajectory
    public static TrajectoryConfig trajectoryConfig(Speeds speed) {
        return trajectoryConfig(speed, false);
    }

    public static TrajectoryConfig trajectoryConfig(Speeds speed, boolean backwards) {
        TrajectoryConfig config;
        if (Config.Trajectories.safetyMode) {
            config = new TrajectoryConfig(Speeds.Safe.velocity, Speeds.Safe.acceleration);
        } else {
            config = new TrajectoryConfig(speed.velocity, speed.acceleration);
        }

        config.setReversed(backwards);
        config.setKinematics(Constants.kDriveKinematics); // Add kinematics to ensure max speed is actually
                                                          // obeyed

        return config;
    }

    public static HolonomicDriveCommand FollowTrajectory(DrivetrainSubsystem drive,
            SwerveTrajectory swerveTrajectory) {
        // SmartDashboard.putNumber("Got to hol command",
        // trajectory.getInitialPose().getX());
        SwerveTrajectory trajectory = swerveTrajectory;
        return new HolonomicDriveCommand(
                trajectory,
                () -> {
                    Pose2d pose = drive.getPose();
                    if (pose == null) {
                        pose = new Pose2d();
                    }
                    return pose;
                },
                new HolonomicDriveController(
                        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
                        // TODO: figure out right PID values for holonomic drive controller if they need
                        // to be figured out
                        new ProfiledPIDController(1, 0, 0,
                                new TrapezoidProfile.Constraints(6.28, 3.14))),
                Constants.kDriveKinematics,
                (p0, p1, p2) -> drive.setVelocities(p0, p1, p2),
                drive);
    }

    public static Pose2d waypoint(double x, double y, double degrees) {
        return new Pose2d(x, y, new Rotation2d(degrees * Constants.PI / 180.0));
    }

    public static Transform2d transform(double x, double y, double degrees) {
        return new Transform2d(new Translation2d(x, y), Rotation2d.fromDegrees(degrees));
    }

    public static Translation2d point(double x, double y) {
        return new Translation2d(x, y);
    }

    public static Trajectory trajectory(TrajectoryConfig config, Pose2d... waypoints) {
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints), config);
    }

    // public void SwerveTrajectory1(Trajectory t, double[] r){
    // Trajectory lateral_trajectory = t;
    // double[] rotational_trajectory = r;
    // }

    // Interior headings determineed automatically
    // public static Trajectory trajectoryCS(TrajectoryConfig config,
    // Pose2d start,
    // Pose2d end,
    // Translation2d... interiorWaypoints) {
    // return TrajectoryGenerator.generateTrajectory(start,
    // Arrays.asList(interiorWaypoints), end, config);
    // }

    public static SwerveTrajectory trajectoryCS(TrajectoryConfig config,
            Pose2d start,
            Pose2d end,
            Translation2d... interiorWaypoints) {
        return trajectoryCS_swerve(config, start, end, interiorWaypoints);
    }

    // Removes the rotation component from trajectory generation and supplants it
    // with rot_trajectory trajectory generation
    public static SwerveTrajectory trajectoryCS_swerve(TrajectoryConfig config, Pose2d start, Pose2d end,
            Translation2d... interiorWaypoints) {

        Pose2d trickStart = waypoint(start.getX(), start.getY(), getStraightPathAngle(start, end, config.isReversed()).getDegrees());
        Pose2d trickEnd = waypoint(end.getX(), end.getY(), getStraightPathAngle(start, end, config.isReversed()).getDegrees());
        Trajectory t = TrajectoryGenerator.generateTrajectory(trickStart, Arrays.asList(interiorWaypoints), trickEnd, config);
        RotationalTrajectory rot_trajectory = new RotationalTrajectory(start.getRotation(), end.getRotation(), t.getTotalTimeSeconds());

        return new SwerveTrajectory(t, rot_trajectory);
    }

    private static Rotation2d getStraightPathAngle(Pose2d start, Pose2d end, boolean isReversed) {
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

    // DEPRICATED
    // public static Trajectory loadTrajectoryFromFile(String filename) {
    // try {
    // Path trajectoryPath =
    // Filesystem.getDeployDirectory().toPath().resolve("paths/" + filename);
    // return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    // DriverStation.reportError("Unable to open trajectory: filename",
    // ex.getStackTrace());
    // return null;
    // }
    // }
}