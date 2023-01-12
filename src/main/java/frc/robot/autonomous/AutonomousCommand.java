package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {
    private DrivetrainSubsystem m_drive;
    private Pose2d m_startingPose;

    public AutonomousCommand(DrivetrainSubsystem drive, Pose2d startingPose, Command... commands) {
        super(commands);

        m_drive = drive;
        m_startingPose = startingPose;
    }

    @Override
    public void initialize() {
        m_drive.resetOdometry(m_startingPose);
        super.initialize();
    }
}