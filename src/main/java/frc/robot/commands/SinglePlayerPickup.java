package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

public class SinglePlayerPickup extends CommandBase {
    DrivetrainSubsystem drive;
    PoseEstimatorSubsystem poseEstimator;
    Pose2d initialPosition;



    public SinglePlayerPickup(DrivetrainSubsystem drive, PoseEstimatorSubsystem poseEstimator) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
    }
    PathPoint startRot;
    PathPoint middleRot;
    PathPoint endRot;
    SwerveAutoBuilder Path;


    @Override
    public void initialize() {
        var currentPose = poseEstimator.getCurrentPose();
        startRot = new PathPoint(currentPose.getTranslation(),currentPose.getRotation());
        middleRot = new PathPoint(currentPose.getTranslation(), new Rotation2d().fromDegrees(100));
        endRot = new PathPoint(currentPose.getTranslation(),new Rotation2d(Math.PI / 2));
        var RotateAround = PathPlanner.generatePath(new PathConstraints(2,1),startRot, endRot);
        HashMap<String, Command> map = new HashMap<>();
        Path = new SwerveAutoBuilder(
                poseEstimator::getCurrentPose,
                poseEstimator::setCurrentPose,
                Constants.DrivetrainConstants.KINEMATICS,
                Constants.PPAutoConstants.translationConstants,
                Constants.PPAutoConstants.rotationConstants,
                drive::setModuleStates,
                map,
                true,
                drive);
        }


    @Override
    public void execute() {


        if (initialPosition == null) {
            initialPosition = poseEstimator.getCurrentPose();
        }


    }

    @Override
    public void end(boolean interrupted) {

    }

     @Override
     public boolean isFinished() {
         return false;
     }
}

