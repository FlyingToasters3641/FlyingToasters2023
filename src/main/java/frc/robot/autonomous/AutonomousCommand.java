package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {
    private DrivetrainSubsystem m_drive;
    private PoseEstimatorSubsystem m_poseEstimator;
    private Pose2d m_startingPose;

    public AutonomousCommand(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Pose2d startingPose, Command... commands) {
        super();
        m_startingPose = startingPose;
        m_drive = drive;
        m_poseEstimator = poseEstimator;
        // TODO: Need to figure out how we are going to set initial pose     addCommands(new InstantCommand(() -> m_drive.resetOdometry(m_startingPose)));
        addCommands(commands);
    }

}