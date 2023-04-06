package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class SnapToScoreCommand extends DriveToPose {
    int bestPosition;
    Boolean goToClosest = true;
    int oneOver = 0;
    double xScoringPosition = 1.8;
    Pose2d[] Positions = new Pose2d[]{
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 0)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 1)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 2)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 3)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 4)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 5)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 6)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 7)), new Rotation2d().fromDegrees(0)),
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 8)), new Rotation2d().fromDegrees(0))
    };
    public SnapToScoreCommand(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Boolean goToClosest) {
        super(drive, poseEstimator, (Pose2d) null);
        poseSupplier = this::getBestPosition;
        this.goToClosest = goToClosest;
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0 * 2), Units.degreesToRadians(720.0 * 2)), 0.01);
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
        SmartDashboard.putNumber("Lining up to score at", bestPosition + 1);
    }


    private Pose2d getBestPosition() {

        int id = ((bestPosition + oneOver) < Positions.length) && ((bestPosition + oneOver) >= 0) ? bestPosition + oneOver : oneOver;
        return Positions[bestPosition + oneOver];
    }
}