package frc.robot.autonomous.pathplanner;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ToastedSwerveAutoBuilder extends BaseAutoBuilder {
  private final SwerveDriveKinematics kinematics;
  private final PIDConstants translationConstants;
  private final PIDConstants rotationConstants;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds;
  private final Subsystem[] driveRequirements;

  private final boolean useKinematics;

  private final double warnNotificationThreshold;
  private final Consumer<CommandBase> warnCallback;
  private final double eStopNotificationThreshold;
  private final Consumer<CommandBase> eStopCallback;

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public ToastedSwerveAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Map<String, Command> eventMap,
      double warnNotificationThreshold,
      Consumer<CommandBase> warnCallback,
      double eStopNotificationThreshold,
      Consumer<CommandBase> eStopCallback,
      Subsystem... driveRequirements) {
    this(
        poseSupplier,
        resetPose,
        translationConstants,
        rotationConstants,
        outputChassisSpeeds,
        eventMap,
        false,
        warnNotificationThreshold,
        warnCallback,
        eStopNotificationThreshold,
        eStopCallback,
        driveRequirements);
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputModuleStates A function that takes raw output module states from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public ToastedSwerveAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      SwerveDriveKinematics kinematics,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      double warnNotificationThreshold,
      Consumer<CommandBase> warnCallback,
      double eStopNotificationThreshold,
      Consumer<CommandBase> eStopCallback,
      Subsystem... driveRequirements) {
    this(
        poseSupplier,
        resetPose,
        kinematics,
        translationConstants,
        rotationConstants,
        outputModuleStates,
        eventMap,
        false,
        warnNotificationThreshold,
        warnCallback,
        eStopNotificationThreshold,
        eStopCallback,
        driveRequirements);
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public ToastedSwerveAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      double warnNotificationThreshold,
      Consumer<CommandBase> warnCallback,
      double eStopNotificationThreshold,
      Consumer<CommandBase> eStopCallback,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

    this.kinematics = null;
    this.translationConstants = translationConstants;
    this.rotationConstants = rotationConstants;
    this.outputModuleStates = null;
    this.outputChassisSpeeds = outputChassisSpeeds;
    this.driveRequirements = driveRequirements;

    this.warnNotificationThreshold = warnNotificationThreshold;
    this.warnCallback = warnCallback;
    this.eStopNotificationThreshold = eStopNotificationThreshold;
    this.eStopCallback = eStopCallback;

    this.useKinematics = false;
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputModuleStates A function that takes raw output module states from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public ToastedSwerveAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      SwerveDriveKinematics kinematics,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      double warnNotificationThreshold,
      Consumer<CommandBase> warnCallback,
      double eStopNotificationThreshold,
      Consumer<CommandBase> eStopCallback,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

    this.kinematics = kinematics;
    this.translationConstants = translationConstants;
    this.rotationConstants = rotationConstants;
    this.outputModuleStates = outputModuleStates;
    this.outputChassisSpeeds = null;
    this.driveRequirements = driveRequirements;

    this.warnNotificationThreshold = warnNotificationThreshold;
    this.warnCallback = warnCallback;
    this.eStopNotificationThreshold = eStopNotificationThreshold;
    this.eStopCallback = eStopCallback;

    this.useKinematics = true;
  }

  @Override
  public CommandBase followPath(PathPlannerTrajectory trajectory) {
    if (useKinematics) {
      return new ToastedPPSwerveControllerCommand(
          trajectory,
          poseSupplier,
          kinematics,
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(rotationConstants),
          outputModuleStates,
          useAllianceColor,
          warnNotificationThreshold,
          warnCallback,
          eStopNotificationThreshold,
          eStopCallback,
          driveRequirements);
    } else {
      return new ToastedPPSwerveControllerCommand(
          trajectory,
          poseSupplier,
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(rotationConstants),
          outputChassisSpeeds,
          useAllianceColor,
          warnNotificationThreshold,
          warnCallback,
          eStopNotificationThreshold,
          eStopCallback,
          driveRequirements);
    }
  }
}
