package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AutoBalanceAlt extends CommandBase {
    DrivetrainSubsystem drive;
    PoseEstimatorSubsystem poseEstimator;
    double pitch = 0.0;
    double correction;
    double prevAngle;
    double roll = 0.0;
    double angularVelocity;
    double angleDegrees;
    final int POSITION_THRESHOLD = 3; // degrees
    double startTime = 0;
    double prevTime = 0;
    final double VELOCITY_THRESHOLD = 5; // degrees/sec 20

    public AutoBalanceAlt(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void execute() {
        double startTime = Timer.getFPGATimestamp();
        double timeDelta = prevTime - startTime;
        var rotation = poseEstimator.getCurrentPose().getRotation();
        pitch = drive.getGyroscopeRotation3d().getY();
        roll = drive.getGyroscopeRotation3d().getX();
        angleDegrees = rotation.getCos() * Math.toDegrees(roll)
                + rotation.getSin() * Math.toDegrees(pitch);

        double angleVelocityDegreesPerSec = (angleDegrees - prevAngle) / timeDelta;

        boolean shouldStop = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > VELOCITY_THRESHOLD)
                || (angleDegrees > 0.0
                && angleVelocityDegreesPerSec < -VELOCITY_THRESHOLD);

        correction = angleDegrees > 0 ? -0.18 : 0.18;

        SmartDashboard.putNumber("Balancing: Robot Rotation", rotation.getDegrees());
        SmartDashboard.putNumber("Balancing: Pitch", pitch);
        SmartDashboard.putNumber("Balancing: Roll", roll);
        SmartDashboard.putNumber("Balancing: Angle (Degrees)", angleDegrees);
        SmartDashboard.putNumber("Balancing: Angular Velocity (Degrees/sec)", angleVelocityDegreesPerSec);
        SmartDashboard.putBoolean("Balancing: Shoud stop?", shouldStop);
        SmartDashboard.putNumber("Balancing: Correction (m/sec)", correction);
        prevAngle = angleDegrees;
        prevTime = startTime;
        if (shouldStop) {
            drive.drive(new ChassisSpeeds(0, 0, 0));

            drive.stop();
        } else {
            drive.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            - correction,
                            0.0,
                            0.0,
                            rotation));
        }

        // drive.drive(new ChassisSpeeds(correction, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
        drive.setWheelsToX();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angleDegrees) < POSITION_THRESHOLD;
    }
}
