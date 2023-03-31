package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class SnapToScoreCommand extends DriveToPose {
    int bestPosition;
    Boolean goToClosest = true;
    int oneOver = 0;
    Pose2d[] Positions = new Pose2d[]{
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 2.9406413367726785 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 3.3698509 - 0.3, new Rotation2d().fromDegrees(0))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 4.00005467 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 4.627886 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 5.0438675 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 5.629004 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 6.3015654 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 6.855354977 - 0.3, new Rotation2d().fromDegrees(-180))),
           PoseEstimatorSubsystem.flipAllianceStatic(new Pose2d(14.8340863, 7.43862497 -1 0.3, new Rotation2d().fromDegrees(-180)))
    };
    public SnapToScoreCommand(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Boolean goToClosest) {
        super(drive, poseEstimator, (Pose2d) null);
        poseSupplier = this::getBestPosition;
        this.goToClosest = goToClosest;
    }

    @Override
    public void initialize() {
        if (goToClosest) {
            var curPose = poseEstimator.getCurrentPose() != null ? poseEstimator.getCurrentPose() : new Pose2d(0, 0, new Rotation2d(0));

            Double prevLength = null;

            for (int i = 0; i < Positions.length; i++) {
                var pose = Positions[i];
                var poseCompare = Math.abs(pose.getTranslation().getDistance(curPose.getTranslation()));
                if (prevLength == null || poseCompare < prevLength) {
                    prevLength = Math.abs(pose.getTranslation().getDistance(curPose.getTranslation()));
                    bestPosition = i + 1;
                }
            }
            SmartDashboard.putNumber("Current Y position:", curPose.getY());
        } else {
            bestPosition = (int) SmartDashboard.getNumber("Lining up to score at", 1);

        }

        bestPosition -= 1;
        SmartDashboard.putNumber("Lining up to score at", bestPosition);
    }


    private Pose2d getBestPosition() {

        int id = ((bestPosition + oneOver) < Positions.length) && ((bestPosition + oneOver) >= 0) ? bestPosition + oneOver : oneOver;
        return Positions[bestPosition + oneOver];
    }
}
