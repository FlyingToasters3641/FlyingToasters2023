package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.function.Supplier;

public class DriveToPose extends CommandBase {
  protected final DrivetrainSubsystem drive;
  protected final PoseEstimatorSubsystem poseEstimator;
  protected Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  protected ProfiledPIDController driveController =
      new ProfiledPIDController(
          2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.inchesToMeters(150.0), Units.inchesToMeters(450.0)), 0.02);
  protected ProfiledPIDController thetaController =
      new ProfiledPIDController(
          5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0), Units.degreesToRadians(720.0)), 0.02);
  protected double driveErrorAbs;
  protected double thetaErrorAbs;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Pose2d pose) {
    this(drive, poseEstimator, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseEstimator = poseEstimator;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = poseEstimator.getCurrentPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    running = true;

    driveController.setTolerance(0.01);
    thetaController.setTolerance(Units.degreesToRadians(1.0));

    // Get current and target pose
    var currentPose = poseEstimator.getCurrentPose();
    var targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
            .getTranslation();
    drive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}
