// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.3937; // FIXME VERIFY --  Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.3937; // FIXME VERIFY --  Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 9; //9
    private final static double kMetersPerInch = 0.0254;

    public static final String CANIVORE_BUS_NAME = "Canivore1";

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 30;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 32;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 31; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 36;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 33;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 41; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 42;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 34;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 38; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0;
    //29.443

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 37;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 35;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 40; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0;
    
    public static final double PI = 3.141592653589793;
    public static final double INCHES_TO_METERS = 0.0254; //NOTE: double INCHES_TO_METERS is equal to the number of meters in a single inch
    public static final double limelightHeightInches = 32; //Set this to the height of the limelight lens (or attached camera lens) from the ground
    public static final double goalHeightInches = 104; //The height of the reflective tape 
    public static final double limelightMountAngleDegrees = 42; //The angle from the horizontal at which the limelight (or attached camera) is mounted
    public static final double ballInitialHeightInches = 32;
    public static final double velocityMultiplier = 1.45; //last set to 1.48

    public static final double kWheelBase = 0.7;
    public static final double kTrackWidth = 0.5;
    public static final SwerveDriveKinematics kDriveKinematics =
    new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    public static double feetToMeters(double feet) {
        return feet * (kMetersPerInch * 12.0);
    }
    
}
