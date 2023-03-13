package test;

import com.pathplanner.lib.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants;
import frc.robot.autonomous.HolonomicDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.beans.Transient;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TrajectoryHelpersTest {

  private final double kT = 0.2;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    var time = new Timer();
    FakeDriveTrainSubsystem m_drive = new FakeDriveTrainSubsystem(
      new Pose2d(0, 0, new Rotation2d(0))
    );
    FakePoseEstimatorSubsystem m_poseEstimator = new FakePoseEstimatorSubsystem(
      m_drive
    );
  }

  private class FakeDriveTrainSubsystem {

    Pose2d position;

    public FakeDriveTrainSubsystem(Pose2d initialPose) {
      position = initialPose;
    }

    public void setVelocities(double p0, double p1, double p2) {
      position =
        new Pose2d(
          position.getX() + (p0 * kT),
          position.getY() + (p1 * kT),
          position.getRotation().plus(new Rotation2d(p2 * kT))
        );
    }
  }

  private class FakePoseEstimatorSubsystem {

    FakeDriveTrainSubsystem drive;

    public FakePoseEstimatorSubsystem(FakeDriveTrainSubsystem drive) {
      this.drive = drive;
    }

    public Pose2d getCurrentPose() {
      return drive.position;
    }
  }

  private static HolonomicDriveCommand FollowTrajectory(
    FakeDriveTrainSubsystem drive,
    FakePoseEstimatorSubsystem poseEstimator,
    PathPlannerTrajectory swerveTrajectory
  ) {
    PathPlannerTrajectory trajectory = swerveTrajectory;
    return new HolonomicDriveCommand(
      trajectory,
      () -> {
        Pose2d pose = poseEstimator.getCurrentPose();
        if (pose == null) {
          pose = new Pose2d();
        }
        return pose;
      },
      new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(
          1,
          0,
          0,
          Constants.AutoConstants.kThetaControllerConstraints
        )
      ),
      Constants.DrivetrainConstants.KINEMATICS,
      (p0, p1, p2) -> drive.setVelocities(p0, p1, p2)
    );
  }

  @Test
  void testAutonomous() {
    PathPlannerTrajectory traj3 = PathPlanner.generatePath(
      new PathConstraints(4, 3),
      new PathPoint(
        new Translation2d(1.0, 1.0),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0),
        2
      ), // position, heading(direction of travel), holonomic rotation, velocity override
      new PathPoint(
        new Translation2d(3.0, 3.0),
        Rotation2d.fromDegrees(45),
        Rotation2d.fromDegrees(-90)
      ), // position, heading(direction of travel), holonomic rotation
      new PathPoint(
        new Translation2d(5.0, 3.0),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(-30) // position, heading(direction of travel), holonomic rotation
      )
    );

    
  }

}
