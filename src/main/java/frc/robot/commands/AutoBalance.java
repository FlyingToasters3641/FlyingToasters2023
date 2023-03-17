package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

    public class AutoBalance extends CommandBase {
        DrivetrainSubsystem drive;
        PoseEstimatorSubsystem poseEstimator;
        double pitch = 0.0;
        double correction;
        double roll = 0.0;


        public AutoBalance(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator) {
            this.drive = drive;
            this.poseEstimator = poseEstimator;
        }

        @Override
        public void execute() {
            correction = roll + pitch < 0 ? 0.1 : 0.1;
            pitch = drive.getGyroscopeRotation3d().getY();
            roll = drive.getGyroscopeRotation3d().getX();
            SmartDashboard.putBoolean("Balancing", true);

            SmartDashboard.putNumber("Pitch", pitch * 180);
            SmartDashboard.putNumber("Roll", roll * 180);

            drive.drive(new ChassisSpeeds(0.1, 0, 0));
        }

        @Override
        public void end(boolean interrupted) {
            drive.drive(new ChassisSpeeds(0,0,0));
        }

        @Override
        public boolean isFinished() {
            return Math.abs(pitch + roll) <= 0.05;
        }
    }

