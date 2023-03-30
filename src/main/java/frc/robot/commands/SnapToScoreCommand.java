package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class SnapToScoreCommand extends DriveToPose {
    int bestPosition;
    int oneOver = 0;
    Pose2d[] Positions = new Pose2d[]{
            new Pose2d(1.81301, 5.098288, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 4.58139, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 3.992152, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 3.330761, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 2.83503, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 2.37096, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 1.654740, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 1.08437, new Rotation2d().fromDegrees(0)),
            new Pose2d(1.81301, 0.47977869, new Rotation2d().fromDegrees(0))
    };
    public SnapToScoreCommand(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, int oneOver) {
        super(drive, poseEstimator, (Pose2d) null);
        poseSupplier = this::getBestPosition;
        this.oneOver = oneOver;
    }

    @Override
    public void initialize() {
        var curPose = poseEstimator.getCurrentPose();
        Double prevLength = null;

        for (int i = 0; i < Positions.length; i++) {
            var pose = Positions[i];
            var poseCompare = Math.abs(pose.getTranslation().getDistance(curPose.getTranslation()));
            if (prevLength == null || poseCompare < prevLength) {
                prevLength = Math.abs(pose.getTranslation().getDistance(curPose.getTranslation()));
                bestPosition = i;
            }
        }
        SmartDashboard.putNumber("Current Y position:", curPose.getY());
        SmartDashboard.putNumber("Lining up to score at", bestPosition);
    }


    private Pose2d getBestPosition() {
        return Positions[bestPosition + oneOver];
    }
}
