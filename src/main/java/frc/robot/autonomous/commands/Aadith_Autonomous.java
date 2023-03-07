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
import frc.robot.subsystems.Swerve;
import static frc.robot.autonomous.TrajectoryHelpers.*;


public class Aadith_Autonomous extends AutonomousCommand {
  
    // private static Pose2d turnAround = new Pose2d(feetToMeters(4.1), 0, Rotation2d.fromDegrees(180));



//     public static final Trajectory pathTurnAround = trajectory(
//             trajectoryConfig(Speeds.Safe),
//             pickUpBall,
//             turnAround
//     );

    

    
    private static Pose2d pointOne = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
    private static Pose2d pointTwo = new Pose2d(1, 0.5, Rotation2d.fromDegrees(120.0));
    private static Pose2d pointThree = new Pose2d(0, 1, Rotation2d.fromDegrees(240.0));
    
    private Swerve m_drive;
    private static SwerveTrajectory trajectoryOne = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), pointOne, pointTwo);
    private static SwerveTrajectory trajectoryTwo = trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), pointTwo, pointThree);
    private static SwerveTrajectory trajectoryThree= trajectoryCS_swerve(trajectoryConfig(Speeds.Safe), pointThree, pointOne);
    // public static final Trajectory pathForward = trajectory(
    //     trajectoryConfig(Speeds.Safe),
    //     start,
    //     pickUpBall
    // );
    
    public Aadith_Autonomous(Swerve drive) {
        super(
                drive,
                pointOne,
                FollowTrajectory(drive, trajectoryOne),
                FollowTrajectory(drive, trajectoryTwo),
                FollowTrajectory(drive, trajectoryThree)


                

                // new InstantCommand(() -> drive.driveArcade(0, 0), drive),
                // new RotateCommand(drive, 180),  // If not working, comment this out and use the brute force method above
                // FollowTrajectory(drive, pathTurnAround),
                // new ShootHighGoalAutonCommand(flywheel, hood, feeder, vision, drive, intake, () -> 10.4)
        );
        m_drive = drive;
    }
}
    