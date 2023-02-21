package frc.robot;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public final int moduleNumber;
    public final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.canBusName);
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, moduleConstants.canBusName);

        configAngleEncoderAndMotor();

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, moduleConstants.canBusName);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            //mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPStoRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            //mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            mDriveMotor.setControl(new VelocityDutyCycle(velocity).withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        //mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        mAngleMotor.setControl(new PositionDutyCycle(Conversions.degreesToFalconRotations(angle.getDegrees(), Constants.Swerve.angleGearRatio)));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconRotationsToDegrees(mAngleMotor.getRotorPosition().getValue(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360.0);
    }

    public void resetToAbsolute(){
        mAngleMotor.setRotorPosition(Conversions.degreesToFalconRotations(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio));

        //double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        //mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoderAndMotor(){        
        var encoderConfig = Robot.ctreConfigs.swerveCanCoderConfig;
        encoderConfig.MagnetSensor.MagnetOffset = -((angleOffset.getDegrees() - 180.0) / 360.0);  // put into range -0.5 to +0.5, CCW+
        angleEncoder.getConfigurator().apply(encoderConfig);

        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());

        var motorConfig = Robot.ctreConfigs.swerveAngleFXConfig;
        motorConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;
        motorConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        // Configure motor/cancoder fused mode
        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        motorConfig.Feedback.RotorToSensorRatio = Constants.Swerve.angleGearRatio;
        mAngleMotor.getConfigurator().apply(motorConfig);

        // resetToAbsolute();
    }

    private void configDriveMotor(){        
        //mDriveMotor.configFactoryDefault();
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        Robot.ctreConfigs.swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        //mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        // mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        // mDriveMotor.setSelectedSensorPosition(0);
        mDriveMotor.setRotorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconRPSToMPS(mDriveMotor.getRotorVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconRotationsToMeters(mDriveMotor.getRotorPosition().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}