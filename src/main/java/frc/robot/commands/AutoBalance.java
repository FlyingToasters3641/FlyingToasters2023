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
        PIDController pid = new PIDController(0.1, 0, 0);
        double pitch = 0.0;
        double roll = 0.0;


        public AutoBalance(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator) {
            this.drive = drive;
            this.poseEstimator = poseEstimator;
            pid.setSetpoint(0);
            pid.setTolerance(0);
        }

        @Override
        public void execute() {
            pitch = drive.getGyroscopeRotation3d().getY();
            roll = drive.getGyroscopeRotation3d().getZ();
            SmartDashboard.putBoolean("Balancing", true);
            SmartDashboard.putNumber("Calculated Balance pid", pid.calculate(roll));
            SmartDashboard.putNumber("Pitch", pitch);
            SmartDashboard.putNumber("Roll", roll);
            drive.drive(new ChassisSpeeds(-pid.calculate(roll), 0, 0));
        }

        @Override
        public void end(boolean interrupted) {
            drive.drive(new ChassisSpeeds(0,0,0));
        }

        @Override
        public boolean isFinished() {
            return pid.atSetpoint();
        }
    }

