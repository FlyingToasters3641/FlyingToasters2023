package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

    public class AutoBalance extends CommandBase {
        DrivetrainSubsystem drive;
        PoseEstimatorSubsystem poseEstimator;
        double pitch = 0.0;
        double correction;
        double prevTime;
        double roll = 0.0;
        double timeCorrection = -Timer.getFPGATimestamp();


        public AutoBalance(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator) {
            this.drive = drive;
            this.poseEstimator = poseEstimator;
        }

        @Override
        public void execute() {
            double time = Timer.getFPGATimestamp() - timeCorrection;
            correction = roll + pitch < 0 ? -0.4 : 0.4;

            correction = Math.abs(correction) < 0.222 ? 0 : correction;
            pitch = drive.getGyroscopeRotation3d().getY();
            roll = drive.getGyroscopeRotation3d().getX();
            SmartDashboard.putBoolean("Balancing", true);

            SmartDashboard.putNumber("Pitch", pitch * 180);
            SmartDashboard.putNumber("Roll", roll * 180);

            drive.drive(new ChassisSpeeds(correction, 0, 0));
            prevTime = time;
        }

        @Override
        public void end(boolean interrupted) {
            drive.drive(new ChassisSpeeds(0,0,0));
        }

        // @Override
        // public boolean isFinished() {
        //     return Math.abs(pitch + roll) <= 0.222;
        // }
    }

