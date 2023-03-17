package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.autonomous.TrajectoryHelpers.Speeds;
import frc.robot.subsystems.Arm.kArm;
import frc.robot.subsystems.swerve.ModuleConfiguration;

public final class Constants {
  public static final double stickDeadband = 0.1;
  public static final String LOGS_FOLDER = "/home/lvuser/";
 
  // Original auton constants
  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // PathPlanner constants
  public static class PPAutoConstants {
    public static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI, 2 / Math.PI);
    public static double THETA_kP = 2;
    public static double THETA_kI = 0.0;
    public static double THETA_kD = 0.0;

    public static double X_kP = 5;
    public static double X_kI = 0.0;
    public static double X_kD = 0.0;

    public static double Y_kP = 5;
    public static double Y_kI = 0.0;
    public static double Y_kD = 0.0;

    public static PIDConstants translationConstants = new PIDConstants(X_kP, X_kI, X_kD);
    public static PIDConstants rotationConstants = new PIDConstants(THETA_kP, THETA_kI, THETA_kD);

    public static PIDController translationController = new PIDController(Constants.PPAutoConstants.X_kP,
        Constants.PPAutoConstants.X_kI, Constants.PPAutoConstants.X_kD);
    public static PIDController strafeController = new PIDController(Constants.PPAutoConstants.Y_kP,
        Constants.PPAutoConstants.Y_kI, Constants.PPAutoConstants.Y_kD);
    public static PIDController thetaController = new PIDController(Constants.PPAutoConstants.THETA_kP,
        Constants.PPAutoConstants.THETA_kI, Constants.PPAutoConstants.THETA_kD);
  }

  public static final class DrivetrainConstants {
    public static final boolean ADD_TO_DASHBOARD = false;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20 + (5 / 8));
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20 + (5 / 8));

    // Pick the longest side of the robot for this and measure outside bumper to
    // outside bumper
    public static final double ROBOT_LENGTH_WIDTH = 0.749; // TODO: Need to do this

    public static final String CANIVORE_BUS_NAME = "Canivore1";

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 37;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 35;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 40;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(25.22);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(164.531250);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 42;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 34;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 38;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(202.675);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(240.292969);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 36;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 33;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 41;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(122.78);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(121.552734);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 30;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 32;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 31;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(292.32419);

    public static final int LEFT_ARM_MOTOR = 26;
    public static final int RIGHT_ARM_MOTOR = 21;
    public static final int EXTENDER_MOTOR = 23; 
    public static final int ARM_POT_CHANNEL = 0; // Port for the potentiometer on the rotation for the arm.
    public static final int EX_POT_CHANNEL = 1; // Port for the extender pot

    public static final int ROLLER_MOTOR = 24;
    public static final int ROLLER_MOTOR_LIMIT = 30; 
    // intake pneumatic
    public static final int EFFECTOR_RETRACTED_CHANNEL = 0;
    public static final int EFFECTOR_EXTENDED_CHANNEL = 1;
    public static final int PNEUMATIC_HUB = 7; 

    public static final int PIGEON_ID = 9;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        ModuleConfiguration.MK4I_L2.getDriveReduction() *
        ModuleConfiguration.MK4I_L2.getWheelDiameter() * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
        /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double DRIVE_kS = 0.28399267231;//0.6716;
    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double DRIVE_kV = 2.190886470165;//2.5913;
    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double DRIVE_kA = 0.178125854;//0.19321;

    public static final double STEER_kP = 0.2; // .3
    public static final double STEER_kI = 0.0;
    public static final double STEER_kD = 0.1; // 0

    public static final double DRIVE_kP = 0.02; // 0.05 6
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;
  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 10.0;
    public static final double Y_RATE_LIMIT = 10.0;
    public static final double ROTATION_RATE_LIMIT = 5.0 * Math.PI;

    public static final double HEADING_MAX_VELOCITY = Math.PI * 2;
    public static final double HEADING_MAX_ACCELERATION = Math.PI * 2;

    public static final double HEADING_kP = 2.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.0;

    public static final double HEADING_TOLERANCE = Math.toRadians(1.5);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    public static final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    public static final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    public static final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
  }

  /* Default settings for trajectories */
  public static final class Trajectories {
    public static final boolean safetyMode = false;
    public static final Speeds defaultSpeed = Speeds.Med;
  }

  public enum IntakePos {
    DEFAULT,
    FLOOR
  }

  public enum ArmPos {
    STORED_POSITION(-56, 0, IntakePos.DEFAULT, false), 
    GROUND_INTAKE_POSITION(-40, 56 /*kArm.EXTENDED_POSITION * 0.4*/, IntakePos.FLOOR, true), 
    SOLO_PLAYERSTATION_PICKUP(-23, 0, IntakePos.DEFAULT, true), 
    DOUBLE_PLAYERSTATION_PICKUP(131, 14, IntakePos.DEFAULT, true), 
    L2_SCORING(142, 0, IntakePos.DEFAULT, false),
    L3_SCORING(142, kArm.EXTENDED_POSITION, IntakePos.DEFAULT, false);

    private double angle;
    private double extendedPosition;
    private IntakePos intakePosition;
    private boolean runIntake;

    private ArmPos(double angle, double extended, IntakePos intakePosition, boolean runIntake) {
      this.angle = angle;
      this.extendedPosition = extended;
      this.intakePosition = intakePosition;
      this.runIntake = runIntake;
    }

    public double getExtended() {
      return extendedPosition;
    }

    public double getAngle() {
      return angle;
    }

    public IntakePos getIntakePosition() {
      return intakePosition;
    }

    public boolean getRunIntake() {
      return runIntake;
    }
  }

}
