package frc.robot.autonomous.commands;

import static frc.robot.autonomous.TrajectoryHelpers.*;

import com.pathplanner.lib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.autonomous.AutonomousCommand;
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
    1,
    0,
    Rotation2d.fromDegrees(0.0)
  );
   static PathPlannerTrajectory inCodeTrajectory = PathPlanner.generatePath(
    Speeds.Safe.Constraints,
    new PathPoint(
      new Translation2d(0, 0),
      Rotation2d.fromDegrees(0),
      Rotation2d.fromDegrees(0)
    ), // position, heading(direction of travel), holonomic rotation, velocity override
    new PathPoint(
      new Translation2d(2, 0),
      Rotation2d.fromDegrees(0),
      Rotation2d.fromDegrees(0) // position, heading(direction of travel), holonomic rotationnew PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30) // position, heading(direction of travel), holonomic rotation
    )
  );

  private DrivetrainSubsystem m_drive;

  public testAuton(
    DrivetrainSubsystem drive,
    PoseEstimatorSubsystem poseEstimator
  ) {
    super(
      drive,
      poseEstimator,
      inCodeTrajectory.getInitialHolonomicPose(),
      FollowTrajectory(drive, poseEstimator, inCodeTrajectory)

    );
    m_drive = drive;
  }
}
