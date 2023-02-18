package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.autonomous.TrajectoryHelpers.Speeds;
import frc.robot.subsystems.Vision.VisionHelpers;
import frc.robot.subsystems.Vision.VisionHelpers.AprilTagConfig;
import frc.lib.math.Conversions;
import frc.lib.math.PolynomialRegression;
import frc.lib.math.Conversions.*;

public final class Constants {
        public static final double stickDeadband = 0.1;

        public static final class Swerve {
                public static final int pigeonID = 9;
                public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

                public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(20 + (5 / 8));
                public static final double wheelBase = Units.inchesToMeters(20 + (5 / 8));
                public static final double wheelCircumference = chosenModule.wheelCircumference;

                /*
                 * Swerve Kinematics
                 * No need to ever change this unless you are not doing a traditional
                 * rectangular/square 4 module swerve
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                /* Module Gear Ratios */
                public static final double driveGearRatio = chosenModule.driveGearRatio;
                public static final double angleGearRatio = chosenModule.angleGearRatio;

                /* Motor Inverts */
                public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
                public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = chosenModule.canCoderInvert;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 25;
                public static final int anglePeakCurrentLimit = 40;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 35;
                public static final int drivePeakCurrentLimit = 60;
                public static final double drivePeakCurrentDuration = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                 * These values are used by the drive falcon to ramp in open loop and closed
                 * loop driving.
                 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
                 */
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* Angle Motor PID Values */
                public static final double angleKP = chosenModule.angleKP;
                public static final double angleKI = chosenModule.angleKI;
                public static final double angleKD = chosenModule.angleKD;
                public static final double angleKF = chosenModule.angleKF;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                /*
                 * Drive Motor Characterization Values
                 * Divide SYSID values by 12 to convert from volts to percent output for CTRE
                 */
                public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
                public static final double driveKV = (1.51 / 12);
                public static final double driveKA = (0.27 / 12);

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
                /** Radians per Second */
                public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

                /* Neutral Modes */
                public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
                public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 {
                        public static final int driveMotorID = 30;
                        public static final int angleMotorID = 32;
                        public static final int canCoderID = 31;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.33);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset, "Canivore1");
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 36;
                        public static final int angleMotorID = 33;
                        public static final int canCoderID = 41;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.78);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset, "Canivore1");
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {
                        public static final int driveMotorID = 42;
                        public static final int angleMotorID = 34;
                        public static final int canCoderID = 38;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(202.675);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset, "Canivore1");
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 37;
                        public static final int angleMotorID = 35;
                        public static final int canCoderID = 40;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(25.22);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset, "Canivore1");
                }
        }

        public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                                  // tuned to specific robot
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

        /* Default settings for trajectories */
        public static final class Trajectories {
                public static final boolean safetyMode = false;
                public static final Speeds defaultSpeed = Speeds.Med;
        }

        public static final AprilTagConfig[] AprilTagConfigs = {
                        new AprilTagConfig(1, new Pose3d(
                                        Units.inchesToMeters(610.77),
                                        Units.inchesToMeters(42.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, Math.PI))),
                        new AprilTagConfig(2, new Pose3d(
                                        Units.inchesToMeters(610.77),
                                        Units.inchesToMeters(108.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, Math.PI))),
                        new AprilTagConfig(3, new Pose3d(
                                        Units.inchesToMeters(610.77),
                                        Units.inchesToMeters(174.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, Math.PI))),
                        new AprilTagConfig(4, new Pose3d(
                                        Units.inchesToMeters(636.96),
                                        Units.inchesToMeters(265.74),
                                        Units.inchesToMeters(27.38),
                                        new Rotation3d(0, 0, Math.PI))),
                        new AprilTagConfig(5, new Pose3d(
                                        Units.inchesToMeters(14.25),
                                        Units.inchesToMeters(265.74),
                                        Units.inchesToMeters(27.38),
                                        new Rotation3d(0, 0, 0))),
                        new AprilTagConfig(6, new Pose3d(
                                        Units.inchesToMeters(40.45),
                                        Units.inchesToMeters(174.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, 0))),
                        new AprilTagConfig(6, new Pose3d(
                                        Units.inchesToMeters(40.45),
                                        Units.inchesToMeters(174.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, 0))),
                        new AprilTagConfig(7, new Pose3d(
                                        Units.inchesToMeters(40.45),
                                        Units.inchesToMeters(108.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, 0))),
                        new AprilTagConfig(8, new Pose3d(
                                        Units.inchesToMeters(40.45),
                                        Units.inchesToMeters(42.19),
                                        Units.inchesToMeters(18.22),
                                        new Rotation3d(0, 0, 0))),
        };

        public static final Pose3d[] cameraPoses = new Pose3d[] {
                        new Pose3d(
                                        Conversions.inchesToMeters(0), // TODO: This is not set for our robot
                                        Conversions.inchesToMeters(0),
                                        Conversions.inchesToMeters(0),
                                        new Rotation3d(0.0, 0.0, Math.PI)
                                                        .rotateBy(new Rotation3d(0.0,
                                                                        Conversions.degreesToRadians(25.0), 0.0)))
        };
        public static final PolynomialRegression xyStdDevModel = new PolynomialRegression(
                        new double[] {
                                        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                                        3.223358, 4.093358, 4.726358
                        },
                        new double[] {
                                        0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.0695, 0.046, 0.1245, 0.0815,
                                        0.193
                        },
                        1);
        public static final PolynomialRegression thetaStdDevModel = new PolynomialRegression(
                        new double[] {
                                        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                                        3.223358, 4.093358, 4.726358
                        },
                        new double[] {
                                        0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068
                        },
                        1);

}
