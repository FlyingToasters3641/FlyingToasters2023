package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.Supplier;

public class HolonomicDriveCommand extends CommandBase {
    private Timer m_timer = new Timer();
    private SwerveTrajectory m_trajectory;
    private Supplier<Pose2d> m_pose;
    private SwerveDriveKinematics m_kinematics;
    private TriConsumer<Double, Double, Double> m_output;
    private ChassisSpeeds m_prevSpeeds = null;
    private HolonomicDriveController m_follower;

    /**
     * Constructs a new HolonomicDrive Command that, when executed, will follow the
     * provided trajectory.
     *
     * @param trajectory               The trajectory to follow.
     * @param pose                     A function that supplies the robot pose - use
     *                                 one of
     *                                 the odometry classes to provide this.
     * @param holonomicDriveController The RAMSETE follower used to follow the
     *                                 trajectory.
     * @param kinematics               The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond    A function that consumes the computed left
     *                                 and right
     *                                 wheel speeds.
     * @param requirements             The subsystems to require.
     */
    public HolonomicDriveCommand(SwerveTrajectory trajectory,
            Supplier<Pose2d> pose,
            HolonomicDriveController holonomicDriveController,
            SwerveDriveKinematics kinematics,
            TriConsumer<Double, Double, Double> outputMetersPerSecond,
            DrivetrainSubsystem driveSubsystem) {
        m_trajectory = ErrorMessages.requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        m_pose = ErrorMessages.requireNonNullParam(pose, "pose", "RamseteCommand");
        m_follower = ErrorMessages.requireNonNullParam(holonomicDriveController, "follower", "RamseteCommand");
        m_kinematics = ErrorMessages.requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
        m_output = ErrorMessages.requireNonNullParam(outputMetersPerSecond, "output", "RamseteCommand");
        addRequirements(driveSubsystem);
    }

    public HolonomicDriveCommand(Trajectory trajectory, RotationalTrajectory rotTrajectory, Supplier<Pose2d> pose,
            HolonomicDriveController holonomicDriveController, SwerveDriveKinematics kdrivekinematics,
            TriConsumer<Double, Double, Double> outputMetersPerSecond, DrivetrainSubsystem drive) {
    }

    @Override
    public void initialize() {
        State initialState = m_trajectory.latTrajectory.sample(0.0);
        m_prevSpeeds = new ChassisSpeeds(initialState.velocityMetersPerSecond,
                0.0, initialState.curvatureRadPerMeter
                        * initialState.velocityMetersPerSecond);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        State targetPosition = m_trajectory.latTrajectory.sample(curTime);
        ChassisSpeeds targetWheelSpeeds = m_follower.calculate(m_pose.get(), targetPosition,
                targetPosition.poseMeters.getRotation());
        double vxMetersPerSecond = targetWheelSpeeds.vxMetersPerSecond;
        double vyMetersPerSecond = targetWheelSpeeds.vyMetersPerSecond;
        double omegaRadiansPerSecond = m_trajectory.rotTrajectory.calculateRotationalVelocity(curTime);
        m_output.accept(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        SmartDashboard.putNumber("Commanded Rotational Power", omegaRadiansPerSecond);
        SmartDashboard.putNumber("Target X", targetPosition.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("Target Y", targetPosition.poseMeters.getTranslation().getY());
        SmartDashboard.putNumber("Target Angle", m_trajectory.rotTrajectory.calculateRotation(curTime));
        m_prevSpeeds = targetWheelSpeeds;

    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.advanceIfElapsed(m_trajectory.latTrajectory.getTotalTimeSeconds());
    }
}