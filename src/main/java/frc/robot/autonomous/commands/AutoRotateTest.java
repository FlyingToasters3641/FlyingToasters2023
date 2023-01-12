package frc.robot.autonomous.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutonomousCommand;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.TrajectoryHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.autonomous.TrajectoryHelpers.*;


public class AutoRotateTest extends AutonomousCommand {
  
    // private static Pose2d turnAround = new Pose2d(feetToMeters(4.1), 0, Rotation2d.fromDegrees(180));



//     public static final Trajectory pathTurnAround = trajectory(
//             trajectoryConfig(Speeds.Safe),
//             pickUpBall,
//             turnAround
//     );

    

    
    private static Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
    private static Pose2d pickUpBall = new Pose2d(0, 6, Rotation2d.fromDegrees(0.0));
    
    private DrivetrainSubsystem m_drive;
    private static SwerveTrajectory pathForward = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), start, pickUpBall);
    // public static final Trajectory pathForward = trajectory(
    //     trajectoryConfig(Speeds.Safe),
    //     start,
    //     pickUpBall
    // );
    
    public AutoRotateTest(DrivetrainSubsystem drive) {
        super(
                drive,
                start,
                FollowTrajectory(drive, pathForward)


                

                // new InstantCommand(() -> drive.driveArcade(0, 0), drive),
                // new RotateCommand(drive, 180),  // If not working, comment this out and use the brute force method above
                // FollowTrajectory(drive, pathTurnAround),
                // new ShootHighGoalAutonCommand(flywheel, hood, feeder, vision, drive, intake, () -> 10.4)
        );
        m_drive = drive;
    }
}
