package frc.robot.autonomous.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutonomousCommand;
import frc.robot.autonomous.TrajectoryHelpers.Speeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.TrajectoryHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.autonomous.TrajectoryHelpers.*;

import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.feetToMeters;
import static frc.robot.Config.INTAKE_SPEED;
import static frc.robot.autonomous.TrajectoryHelpers.*;

import com.ctre.phoenix.Logger;

public class AntiSabatogeClimberFourBall extends AutonomousCommand {
  public static final Pose2d start = waypoint((10.4521), (3.7084 + 0.381), 180);
  public static final Pose2d startbackup = waypoint((10.5029 + 0.152), (3.7084 + 0.381), 180);
  public static final Pose2d turn = waypoint((10.505), (3.7084), 175);
  public static final Pose2d ball_1 = waypoint(14.226804, 3.602290, 135);
  public static final Pose2d ball_2 = waypoint(15.241903 + (-1 * 0.07212) + 0.0359209 + (2 * 0.0179578),
      6.821174 + (-1 * 0.07212) + 0.0359209 + (2 * 0.0179578), 45);
  public static final Pose2d shoot_2 = waypoint(11.435092, 6.214380 - 0.127 - 0.0762, 220);
  public static final Pose2d backupPosition = waypoint(14.654731, 6.638532 - 1.2, 130);

  private static SwerveTrajectory startbackupTrajectory = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe, true),
      start, startbackup);
  private static SwerveTrajectory path1_alt = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe, true), start, ball_1,
      point(12.066793, 4.233992));
  private static SwerveTrajectory path2_alt = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), ball_1, ball_2);
  private static SwerveTrajectory backup = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe, true), ball_2,
      backupPosition);
  private static SwerveTrajectory path3_alt = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), backupPosition,
      shoot_2);

  private DrivetrainSubsystem m_drive;

  public AntiSabatogeClimberFourBall(DrivetrainSubsystem drive) {
    super(
        drive,
        path1_alt.latTrajectory.getInitialPose(),
        FollowTrajectory(drive, startbackupTrajectory),
        new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                FollowTrajectory(drive, path1_alt)),
            new WaitCommand(0.1)),

        new ParallelDeadlineGroup(
            FollowTrajectory(drive, path2_alt)),

        new ParallelDeadlineGroup(
            new WaitCommand(2)),

        new ParallelDeadlineGroup(
            FollowTrajectory(drive, backup)),

        new ParallelDeadlineGroup(
            FollowTrajectory(drive, path3_alt)));
    m_drive = drive;
  }
}
