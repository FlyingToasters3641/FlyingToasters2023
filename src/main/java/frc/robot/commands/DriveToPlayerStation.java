package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class DriveToPlayerStation extends DriveToPose {
  public static final Pose2d singleSubstationPose = new Pose2d(
      FieldConstants.LoadingZone.singleSubstationTranslation.plus(new Translation2d(0.0, -0.7)),
      Rotation2d.fromDegrees(90.0));
  public static final double doubleSubstationX = FieldConstants.LoadingZone.doubleSubstationX - 0.4;

  public DriveToPlayerStation(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator,
      Supplier<Boolean> useDouble) {
    super(
        drive,
        poseEstimator,
        () -> {
          var nearestTarget = useDouble.get()
              ? AllianceFlipUtil.apply(
                  new Pose2d(doubleSubstationX, poseEstimator.getCurrentPose().getY(), Rotation2d.fromDegrees(180.0)))
              : AllianceFlipUtil.apply(singleSubstationPose);
          return nearestTarget;
        });
  }
}
