package frc.robot.subsystems;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.function.Consumer;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionHelpers.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.NorthStarInputs;
import frc.robot.util.FieldConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.005, 0.005, 0.0009);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.04, 0.04,5);

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();
//    private final Field2d fieldOdometry2d = new Field2d();
    private final Field2d fieldVision2d = new Field2d();
    private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

    private final ArrayList<Double> xValues = new ArrayList<>();
    private final ArrayList<Double> yValues = new ArrayList<>();
    Consumer<AprilTagMeasurement> addData;
    private final AprilTagSubsystem NorthStarEstimator;



    public PoseEstimatorSubsystem(/* PhotonCamera photonCamera,*/ DrivetrainSubsystem drivetrainSubsystem) {

        this.drivetrainSubsystem = drivetrainSubsystem;
        //PhotonPoseEstimator photonPoseEstimator;
        try {
            var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            layout.setOrigin(originPosition);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            //photonPoseEstimator = null;
        }
        //this.photonPoseEstimator = photonPoseEstimator;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.KINEMATICS,
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
       // tab.add("Odometry Field", fieldOdometry2d).withPosition(2,0).withSize(6,4);
        tab.add("Vision Field", fieldVision2d).withPosition(2,0).withSize(6,4);

        //addData = measure -> poseEstimator.addVisionMeasurement(flipAlliance(measure.getPose().toPose2d()), measure.getTimestamp());

        addData = measure -> {
            var visionPose = measure.getPose().toPose2d();
            if (DriverStation.getAlliance() == Alliance.Red) {
                visionPose = flipAlliance(visionPose);
            }
      
            fieldVision2d.setRobotPose(visionPose);
            if (!(!(DriverStation.isTeleopEnabled()) && (RobotContainer.selectedAutonomous.equals("2GPBarrier") || RobotContainer.selectedAutonomous.equals("OneConeBalance")))) {
                poseEstimator.addVisionMeasurement(visionPose, measure.getTimestamp());
            }
        };


        NorthStarEstimator = new AprilTagSubsystem(
                addData,
                 new NorthStarInputs("NorthStarLeft", new Pose3d(-0.198938789, 0.270769403, 0.628645808, new Rotation3d(0,0,2.79252665359))),
                 new NorthStarInputs("NorthStarRight", new Pose3d(-0.198938789, -0.270769403, 0.628645808, new Rotation3d(0,0,0.349066 + Math.PI)))

        );
    }


    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     *
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        //var fieldTags = photonPoseEstimator.getFieldTags();
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                //fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
                originPosition = OriginPosition.kBlueAllianceWallRightSide;
                break;
            case Red:
                //fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
                originPosition = OriginPosition.kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }
        if (allianceChanged) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag may have been seen and the tags are all relative to the
            // coordinate system, the estimated pose
            // needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(poseEstimator.getEstimatedPosition());
            setCurrentPose(newPose);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions());


        Pose2d dashboardPose = getCurrentPose();
        if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = flipAlliance(dashboardPose);
        }
        field2d.setRobotPose(dashboardPose);
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates
     * for each alliance.
     *
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS),
                new Rotation2d(Math.PI)));
    }

    public static Pose2d flipAllianceStatic(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS),
                new Rotation2d(Math.PI)));
    }

    private Pose2d clampToDrivableArea(Pose2d pose) {

        return pose;
    }

    public void addTrajectory(PathPlannerTrajectory traj) {
        field2d.getObject("Trajectory").setTrajectory(traj);
    }

    /**
     * Resets the holonomic rotation of the robot (gyro last year)
     * This would be used if Apriltags are not getting accurate pose estimation
     */
    public void resetHolonomicRotation() {
        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(0),
                drivetrainSubsystem.getModulePositions(),
                getCurrentPose());
    }

    public void resetPoseRating() {
        xValues.clear();
        yValues.clear();
    }

}