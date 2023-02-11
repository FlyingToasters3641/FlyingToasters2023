package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class AutonomousCommand extends SequentialCommandGroup {
    private Swerve m_drive;
    private Pose2d m_startingPose;

    public AutonomousCommand(Swerve drive, Pose2d startingPose, Command... commands) {
        super();
        m_startingPose = startingPose;
        m_drive = drive;
        addCommands(new InstantCommand(() -> m_drive.resetOdometry(m_startingPose)));
        addCommands(commands);
    }

}