package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AutoBalanceAlt extends CommandBase {
    DrivetrainSubsystem drive;
    PoseEstimatorSubsystem poseEstimator;
    double pitch = 0.0;
    double correction;
    double roll = 0.0;
    double angularVelocity;
    double angleDegrees;
    final int POSITION_THRESHOLD = 3; // degrees
    final int VELOCITY_THRESHOLD = 8; // degrees/sec

    public AutoBalanceAlt(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void execute() {
        var rotation = poseEstimator.getCurrentPose().getRotation();
        pitch = drive.getGyroscopeRotation3d().getY();
        roll = drive.getGyroscopeRotation3d().getX();
        angleDegrees = rotation.getCos() * Math.toDegrees(roll)
                + rotation.getSin() * Math.toDegrees(pitch);

        double angleVelocityDegreesPerSec = rotation.getCos() * drive.getRollDegreesSec()
                + rotation.getSin() * drive.getPitchDegreesSec();

        boolean shouldStop = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > VELOCITY_THRESHOLD)
                || (angleDegrees > 0.0
                        && angleVelocityDegreesPerSec < -VELOCITY_THRESHOLD);

        correction = angleDegrees > 0 ? -0.4 : 0.4;

        SmartDashboard.putNumber("Balancing: Robot Rotation", rotation.getDegrees());
        SmartDashboard.putNumber("Balancing: Pitch", pitch);
        SmartDashboard.putNumber("Balancing: Roll", roll);
        SmartDashboard.putNumber("Balancing: Angle (Degrees)", angleDegrees);
        SmartDashboard.putNumber("Balancing: Angular Velocity (Degrees/sec)", angleVelocityDegreesPerSec);
        SmartDashboard.putBoolean("Balancing: Shoud stop?", shouldStop);
        SmartDashboard.putNumber("Balancing: Correction (m/sec)", correction);

        if (shouldStop) {
            drive.stop();
        } else {
            drive.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            correction,
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
