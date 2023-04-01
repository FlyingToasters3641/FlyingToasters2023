package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.Alert;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.trajectory.CustomHolonomicDriveController;
import frc.robot.util.trajectory.CustomTrajectoryGenerator;
import frc.robot.util.trajectory.RotationSequence;
import frc.robot.util.trajectory.Waypoint;

import java.util.List;
import java.util.function.Supplier;

public class DriveTrajectory extends CommandBase {
  private static final Alert generatorAlert =
      new Alert("Failed to generate trajectory, check constants.", AlertType.ERROR);

  private static boolean supportedRobot = true;
  private static double maxVelocityMetersPerSec = Units.inchesToMeters(160.0);
  private static double maxAccelerationMetersPerSec2 = Units.inchesToMeters(105.0);
  private static double maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(150.0);;

  private final PIDController xController = new PIDController(6.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(6.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(8.0, 0.0, 0.0);

  private final CustomHolonomicDriveController customHolonomicDriveController =
      new CustomHolonomicDriveController(xController, yController, thetaController);

  private final DrivetrainSubsystem drive;
  private final PoseEstimatorSubsystem poseEstimator;
  private final Timer timer = new Timer();

  private Supplier<List<Waypoint>> waypointsSupplier = null;
  private Supplier<List<TrajectoryConstraint>> constraintsSupplier = null;
  private Supplier<Double> startVelocitySupplier = null;
  private CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

  /** Creates a DriveTrajectory command with a dynamic set of waypoints. */
  public DriveTrajectory(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, Supplier<List<Waypoint>> waypointsSupplier) {
    this(drive, poseEstimator, waypointsSupplier, () -> List.of(), () -> 0.0);
  }

  /** Creates a DriveTrajectory command with a dynamic set of waypoints and constraints. */
  public DriveTrajectory(
      DrivetrainSubsystem drive,
      PoseEstimatorSubsystem poseEstimator,
      Supplier<List<Waypoint>> waypointsSupplier,
      Supplier<List<TrajectoryConstraint>> constraintsSupplier,
      Supplier<Double> startVelocitySupplier) {
    this.drive = drive;
    this.poseEstimator = poseEstimator;
    addRequirements(drive);
    this.waypointsSupplier = waypointsSupplier;
    this.constraintsSupplier = constraintsSupplier;
    this.startVelocitySupplier = startVelocitySupplier;
  }

  /** Creates a DriveTrajectory command with a static set of waypoints. */
  public DriveTrajectory(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator, List<Waypoint> waypoints) {
    this(drive, poseEstimator, waypoints, List.of(), 0.0);
  }

  /** Creates a DriveTrajectory command with a static set of waypoints and constraints. */
  public DriveTrajectory(
      DrivetrainSubsystem drive,
      PoseEstimatorSubsystem poseEstimator,
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocity) {
    this.drive = drive;
    this.poseEstimator = poseEstimator;
    addRequirements(drive);
    generate(waypoints, constraints, startVelocity, true);
  }

  /** Generates the trajectory. */
  private void generate(
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocity,
      boolean alertOnFail) {
    // Set up trajectory configuration
    TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(Constants.DrivetrainConstants.KINEMATICS)
            .setStartVelocity(startVelocity)
            .setEndVelocity(0.0)
            .addConstraint(
                new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2))
            .addConstraints(constraints);

    // Generate trajectory
    customGenerator = new CustomTrajectoryGenerator(); // Reset generator
    try {
      customGenerator.generate(config, waypoints);
    } catch (Exception exception) {
      if (supportedRobot && alertOnFail) {
        generatorAlert.set(true);
        exception.printStackTrace();
      }
    }
  }

  @Override
  public void initialize() {
    // Generate trajectory if supplied
    if (waypointsSupplier != null || constraintsSupplier != null) {
      generate(
          waypointsSupplier.get(), constraintsSupplier.get(), startVelocitySupplier.get(), false);
    }
    // // Log trajectory
    // SmartDashboard.putStringArray("Trajectory Initial trajectory",
    //         customGenerator.getDriveTrajectory().getStates().stream()
    //             .map(state -> AllianceFlipUtil.apply(state.poseMeters))
    //             .toArray());

    // Reset all controllers
    timer.reset();
    timer.start();
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    // Exit if trajectory generation failed
    if (customGenerator.getDriveTrajectory().getStates().size() <= 1) {
      return;
    }

    // Get setpoint
    Trajectory.State driveState =
        AllianceFlipUtil.apply(customGenerator.getDriveTrajectory().sample(timer.get()));
    RotationSequence.State holonomicRotationState =
        AllianceFlipUtil.apply(customGenerator.getHolonomicRotationSequence().sample(timer.get()));
    // SmartDashboard.putString("Trajectory: Setpoint",
    //         new Pose2d(driveState.poseMeters.getTranslation(), holonomicRotationState.position).toString());

    // Calculate velocity
    ChassisSpeeds nextDriveState =
        customHolonomicDriveController.calculate(
            poseEstimator.getCurrentPose(), driveState, holonomicRotationState);
    drive.drive(nextDriveState);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }
}
