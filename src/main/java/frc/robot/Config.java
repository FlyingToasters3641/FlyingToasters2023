package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.lib.Units.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.autonomous.TrajectoryHelpers.Speeds;

public final class Config {
    // General hardware
    public static final int REVPH_CAN_ID = 5; // TODO: Confirm - we set to 5 when updating firmware but should recheck
    private static final int FALCON_EPR = 2048; // Edges per revolution for falcon 500 internal sensor

    // Drive subsystem hardware IDs
    public static final int MAX_DRIVE_CURRENT = 55;
    public static final double DRIVETRAIN_RATIO = 7.44 / 1; // Gear ratio of drivetrain
    public static final double WHEEL_RADIUS = (13.4 / Math.PI) / 2.0;
    public static final double WHEEL_CIRCUMFERENCE = (13.4 / Math.PI) * INCHES_TO_METERS * PI; //13.75-measured with Mr.Weber

    // Distance (in meters) driven after one encoder pulse
    public static final double METERS_PER_ENCODER_EDGE = WHEEL_CIRCUMFERENCE / (FALCON_EPR * DRIVETRAIN_RATIO);

    // velocity (in meters/s) per velocity unit
    public static final double ENCODER_RATE_TO_VELOCITY = METERS_PER_ENCODER_EDGE * 10.0;

    // Ball feeder hardware IDs
    public static final int FEEDER_INDEXER_MOTOR = 22; // Lower feeder
    public static final int FEEDER_TOWER_MOTOR = 21; // Upper feeder
    public static final int BALL_TOWER_SENSOR_DIO_CHANNEL = 1;
    public static final int BALL_INDEXER_SENSOR_DIO_CHANNEL = 3;
    // public static final double GET_BALL_SPEED = 0.25;
    // public static final double RETRACT_SPEED = -0.3;
    // public static final double FEED_BALL_SPEED = 1.0;
    public static final double FEEDER_INDEXER_SPEED = 0.6;
    //public static final double FEEDER_INDEXER_SPEED = 1;
    public static final double FEEDER_TOWER_SPEED = 0.6;
    public static final double PULSE_SPEED = -0.2;
    public static final double PULSE_TIME = 3.0;
    public static final double PULSE_DURATION = 0.25;

    // Intake
    public static final int LEFT_INTAKE_MOTOR = 20;
    public static final int RIGHT_INTAKE_MOTOR = 23;
    public static final int EXTENDER_FORWARD_CHANNEL = 0;
    public static final int EXTENDER_REVERSE_CHANNEL = 1;
    public static final double INTAKE_SPEED = 0.5;
    public static final double INTAKE_RETRACT_TIME_SECONDS = 0.5;
    public static final double INTAKE_EJECT_SPEED = -0.3;

    // Launcher
    public static final int FLYWHEEL_MASTER_MOTOR = 15; // Left shooter
    public static final int FLYWHEEL_FOLLOWER_MOTOR = 16; // Right shooter
    public static final double FLYWHEEL_VELOCITY_TOLERANCE = 0.5;

    public static final double FLYWHEEL_RATIO = 48.0 / 24.0 ; // TODO: Confirm       Gear ratio of drivetrain
    public static final double FLYWHEEL_WHEEL_CIRCUMFERENCE = 4.0 * INCHES_TO_METERS * PI;
    public static final double FLYWHEEL_METERS_PER_ENCODER_EDGE = FLYWHEEL_WHEEL_CIRCUMFERENCE / (FALCON_EPR * FLYWHEEL_RATIO); // Distance (in meters) driven after one encoder pulse
    public static final double FLYWHEEL_ENCODER_RATE_TO_VELOCITY = FLYWHEEL_METERS_PER_ENCODER_EDGE * 10.0; // Mult by 10.0 to get m/s because Talon encoders report velocity in sensor ticks per 100ms (100ms X 10 == 1000ms == 1 second)
    public static final double FLYWHEEL_kV = 0.058 * 12;
    public static final double FLYWHEEL_kP = 0.05;

    public static final double CLOSE_HIGH_GOAL_SPEED = 10.1; // TODO: Set in meters/s
    public static final double CLOSE_LOW_GOAL_SPEED = 4.2;

    // Launcher hood
    public static final int HOOD_FORWARD_CHANNEL = 2;
    public static final int HOOD_REVERSE_CHANNEL = 3;

    // Climber
    public static final int CLIMBER_MOTOR = 12;
    public static final int CLIMBER_RETRACT_LIMIT_SWITCH = 0;
    public static final int ARM_CYLINDER_FORWARD_CHANNEL = 4;
    public static final int ARM_CYLINDER_REVERSE_CHANNEL = 5;
    public static final double CLIMBER_kP = 0.03;
    public static final double CLIMBER_START_HOLD_POSITION = -7000.0;
    public static final double CLIMBER_EXTENDED_POSITION_MID_BAR = 138208.0;
    public static final double CLIMBER_EXTENDED_POSITION_HIGH_BAR = 220000.0;
    public static final double CLIMBER_EXTENDED_POSITION_HIGHEST_BAR = 177009.0;
    public static final double CLIMBER_POSITION_TOLERANCE = 6000;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    public static final boolean fieldRelativeEnabled = true;

    public static final class Characterization {
        // public static final double kS = 0.465; // Volts
        // public static final double kV = 2.18; // Volts per m/s (Volt seconds per meter)
        // public static final double kA = 0.118; // Volts per m/s^2 (Volt seconds^2 per meter)
        public static final double kS = 0.098517; //0.35 //0.71817; // 0.72818; // Volts
        public static final double kV = 2.3163;//0.95354 // 0.77832; // Volts per m/s (Volt seconds per meter)
        public static final double kA = 0.15711;//0.055253 // 0.10349; // Volts per m/s^2 (Volt seconds^2 per meter)
        public static final double trackWidth = inchesToMeters((26 + (3 / 16))); // Meters

        // public static final double kP = 0.17; // From characterization: 0.186 // for position feedback loop: 0.0443
        // public static final double kD = 0.0; // for position feedback loop: 18.8
        public static final double kP = 0.11; // 0.17; // From characterization: 0.186 // for position feedback loop: 0.0443
        public static final double kD = 0.01; // for position feedback loop: 18.8

       // public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(FILLME);
    }

    public static final class Trajectories {
        public static final boolean safetyMode = false;
        public static final Speeds defaultSpeed = Speeds.Med;
    }
}
