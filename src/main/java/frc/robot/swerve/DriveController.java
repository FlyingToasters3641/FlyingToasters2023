package frc.robot.swerve;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
