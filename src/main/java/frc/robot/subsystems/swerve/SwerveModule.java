package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final SwerveSpeedController driveController;
  private final SwerveSteerController steerController;

  public SwerveModule(SwerveSpeedController driveController, SwerveSteerController steerController) {
    this.driveController = driveController;
    this.steerController = steerController;
  }

  /**
   * Returns the drive velocity in meters per second
   * 
   * @return drive velocity in meters per second
   */
  public double getDriveVelocity() {
    return driveController.getStateVelocity();
  }

  public Rotation2d getSteerAngle() {
    return steerController.getStateRotation();
  }

  public void setDesiredState(SwerveModuleState moduleState) {
    var optimizedState = SwerveModuleState.optimize(moduleState, getSteerAngle());
    driveController.setReferenceVelocity(optimizedState.speedMetersPerSecond);
    steerController.setDesiredRotation(optimizedState.angle);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveController.getStatePosition(), getSteerAngle());
  }

  public WPI_TalonFX getDriveMotor() {
    return driveController.getMotor();
  }

  public WPI_TalonFX getSteerMotor() {
    return steerController.getMotor();
  }

  /**
   * Sets the neutral mode for the drive and steer motors
   * 
   * @param neutralMode neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    steerController.setNeutralMode(neutralMode);
    driveController.setNeutralMode(neutralMode);
  }

  /**
   * Reseeds to Talon FX motor offset from the CANCoder. Workaround for "dead
   * wheel"
   */
  public void reseedSteerMotorOffset() {
    steerController.configMotorOffset(false);
  }

  public Rotation2d getCancoderAbsoluteAngle() {
    return Rotation2d.fromRadians(steerController.getAbsoluteAngle());
  }
}