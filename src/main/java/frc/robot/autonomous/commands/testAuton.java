package frc.robot.autonomous.commands;

import static frc.robot.autonomous.TrajectoryHelpers.*;

import com.pathplanner.lib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.autonomous.AutonomousCommand;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class testAuton extends AutonomousCommand {

  // private static Pose2d turnAround = new Pose2d(feetToMeters(4.1), 0, Rotation2d.fromDegrees(180));

  //     public static final Trajectory pathTurnAround = trajectory(
  //             trajectoryConfig(Speeds.Safe),
  //             pickUpBall,
  //             turnAround
  //     );

  private static Pose2d pointOne = new Pose2d(
    0,
    0,
    Rotation2d.fromDegrees(0.0)
  );
  private static Pose2d pointTwo = new Pose2d(
    1.82,
    0,
    Rotation2d.fromDegrees(90.0)
  );
  static PathPlannerTrajectory traj3 = PathPlanner.generatePath(
    new PathConstraints(4, 3),
    new PathPoint(
      new Translation2d(1.0, 1.0),
      Rotation2d.fromDegrees(0),
      Rotation2d.fromDegrees(0),
      2
    ), // position, heading(direction of travel), holonomic rotation, velocity override
    new PathPoint(
      new Translation2d(3.0, 3.0),
      Rotation2d.fromDegrees(45),
      Rotation2d.fromDegrees(-90) // position, heading(direction of travel), holonomic rotationnew PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30) // position, heading(direction of travel), holonomic rotation
    )
  );
  //private static Pose2d pointThree = new Pose2d(0, 1, Rotation2d.fromDegrees(0.0));

  private DrivetrainSubsystem m_drive;
  private static SwerveTrajectory trajectoryOne = trajectoryCS_swerve(
    trajectoryConfig(Speeds.Safe),
    pointOne,
    pointTwo
  );

  // private static SwerveTrajectory trajectoryTwo = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), pointTwo, pointThree);
  // private static SwerveTrajectory trajectoryThree= trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), pointThree, pointOne);
  // public static final Trajectory pathForward = trajectory(
  //     trajectoryConfig(Speeds.Safe),
  //     start,
  //     pickUpBall
  // );

  public testAuton(
    DrivetrainSubsystem drive,
    PoseEstimatorSubsystem poseEstimator
  ) {
    super(
      drive,
      poseEstimator,
      pointOne,
      FollowTrajectory(drive, poseEstimator, traj3)
      //FollowTrajectory(drive, trajectoryTwo),
      //FollowTrajectory(drive, trajectoryThree)

      // new InstantCommand(() -> drive.driveArcade(0, 0), drive),
      // new RotateCommand(drive, 180),  // If not working, comment this out and use the brute force method above
      // FollowTrajectory(drive, pathTurnAround),
      // new ShootHighGoalAutonCommand(flywheel, hood, feeder, vision, drive, intake, () -> 10.4)
    );
    m_drive = drive;
  }
}
