package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.FieldConstants;

public class SnapToScoreCommand extends DriveToPose {
    int bestPosition;
    Boolean goToClosest = true;
    int oneOver = 0;
    double xScoringPosition = 1.73;
    double valueConst = 8;

    Pose2d[] Positions = new Pose2d[]{  // .5588
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 0)), new Rotation2d().fromDegrees(0)),  // .512826 // -2
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 1)), new Rotation2d().fromDegrees(0)),  // .512826 + .5588 = 1.071626 // -2
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 2)), new Rotation2d().fromDegrees(0)),  // 1.630426 // -2
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 3)), new Rotation2d().fromDegrees(0)),  // 2.189226 // -2
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 4)), new Rotation2d().fromDegrees(0)),  // 2.748026 // -2
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 5)), new Rotation2d().fromDegrees(0)),  // 3.306826
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 6)), new Rotation2d().fromDegrees(0)),  // 3.865626 // -2
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 7)), new Rotation2d().fromDegrees(0)),  // 4.424426
           new Pose2d(xScoringPosition, Units.inchesToMeters(20.19 + (22.0 * 8)), new Rotation2d().fromDegrees(0))  // 4.983226 // +3
    };
    public SnapToScoreCommand(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Boolean goToClosest) {
        super(drive, poseEstimator, (Pose2d) null);
        poseSupplier = this::getBestPosition;
        driveController =
                new ProfiledPIDController(
                        5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.inchesToMeters(150.0), Units.inchesToMeters(450.0)), 0.02);
        thetaController =
                new ProfiledPIDController(
                        3.6, 0.018, 0.0, new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0), Units.degreesToRadians(720.0)), 0.02);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.goToClosest = goToClosest;
       // ProfiledPIDController thetaController =
         //       new ProfiledPIDController(
           //             5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0 * 2), Units.degreesToRadians(720.0 * 2)), 0.01);
    }

    @Override
    public void initialize() {
        if (goToClosest) {
            var curPose = poseEstimator.getCurrentPose() != null ? poseEstimator.getCurrentPose() : new Pose2d(0, 0, new Rotation2d(0));

            Double prevLength = null;

            for (int i = 0; i < Positions.length; i++) {
                var pose = flipYIfNeeded(Positions[i]);
                var poseCompare = Math.abs(pose.getTranslation().getDistance(curPose.getTranslation()));
                if (prevLength == null || poseCompare < prevLength) {
                    prevLength = Math.abs(pose.getTranslation().getDistance(curPose.getTranslation()));
                    bestPosition = i;
                }
            }
            SmartDashboard.putNumber("Current Y position:", curPose.getY());
        } else {
            bestPosition = (int) SmartDashboard.getNumber("Lining up to score at", 1) - 1;

        }


        SmartDashboard.putNumber("Lining up to score at", bestPosition + 1);
    }

    private Pose2d flipYIfNeeded(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Pose2d(pose.getX(), FieldConstants.FIELD_WIDTH_METERS - pose.getY(), pose.getRotation());
        } else {
            return pose;
        }
    }
    private Pose2d getBestPosition() {
//        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
//            for (int i = 0; i < Positions.length; i++) {
//                Positions[i] = new Pose2d(Positions[i].getX(), FieldConstants.FIELD_WIDTH_METERS - Positions[i].getY(), Positions[i].getRotation());
//            }
//        }
        int id = ((bestPosition + oneOver) < Positions.length) && ((bestPosition + oneOver) >= 0) ? bestPosition + oneOver : oneOver;
        return flipYIfNeeded(Positions[bestPosition + oneOver]);
    }
}
