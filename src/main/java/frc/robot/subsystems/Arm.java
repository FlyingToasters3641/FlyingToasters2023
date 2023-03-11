package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.lib.util.MotorHelper;
import frc.lib.util.TunableNumber;
import frc.lib.util.COTSFalconSwerveConstants.driveGearRatios;

public class Arm extends SubsystemBase {
    private SparkMaxPIDController m_leftMotorPid;
    private CANSparkMax m_leftArmMotor;
    private CANSparkMax m_rightArmMotor;
    private CANSparkMax m_extenderMotor;
    private AnalogPotentiometer m_pot;
    private AnalogPotentiometer m_exPot;
    private ArmFeedforward m_armFeedforward;
    double extenderTarget;

    private Double m_targetArmPosition = null;

    public static final class kArm {
        public static final double GEAR_RATIO = 90 / 1;
        public static final double KP = 0.0004; //0.0025; //0.04;//0.090071;//0.0001;
        public static final double KI = 0;//0.0001;
        public static final double KD = 0;//0.000015; //0.017546;//0.0;
        public static final double KF = 0.0;
        public static final double KS = 0.078474;//0.11814;
        public static final double KG = 0.53836;//0.49202;
        public static final double KV = 0.032529;//0.034012;
        public static final double KA = 0.00078541;//0.0020898;
        public static final int LEFT_CURRENT_LIMIT = 39;
        public static final int RIGHT_CURRENT_LIMIT = 39;
        public static final double MIN_PID_OUTPUT = -0.3;
        public static final double MAX_PID_OUTPUT = 0.3;
        public static final double SMART_MOTION_MAX_VELOCITY = 1500; // rpm
        public static final double SMART_MOTION_MAX_ACCEL = 1000;
        public static final double SMART_MOTION_MIN_VELOCITY = 0; // rpm

        // values for Extender
        public static final double GEAR_RATIO_EX = 9 / 1;
        public static final double EX_KP = 0.015;
        public static final double EX_KI = 0.0;
        public static final double EX_KD = 0.0;
        public static final double EX_KF = 0.0001;
        public static final double EXTENDED_POSITION = 26.23; // TODO: measure analog pot for extender.

        public static final double ERROR = 5.0; // degrees
        public static final double MIN_POSITION = -59.0; // degrees
        public static final double MAX_POSITION = 180; // degrees
    }

    public Arm() {
        m_armFeedforward = new ArmFeedforward(
                kArm.KS,
                kArm.KG,
                kArm.KV,
                kArm.KA);

        m_leftArmMotor = MotorHelper.createSparkMax(
                DrivetrainConstants.LEFT_ARM_MOTOR,
                MotorType.kBrushless,
                false,
                kArm.LEFT_CURRENT_LIMIT,
                IdleMode.kBrake,
                kArm.KP,
                kArm.KI,
                kArm.KD,
                kArm.KF);

        m_rightArmMotor = MotorHelper.createSparkMax(
                DrivetrainConstants.RIGHT_ARM_MOTOR,
                MotorType.kBrushless,
                true,
                kArm.RIGHT_CURRENT_LIMIT,
                IdleMode.kBrake,
                kArm.KP,
                kArm.KI,
                kArm.KD,
                kArm.KF);

        m_extenderMotor = MotorHelper.createSparkMax(
                DrivetrainConstants.EXTENDER_MOTOR,
                MotorType.kBrushless,
                true,
                kArm.RIGHT_CURRENT_LIMIT, // TODO: verify current limit
                IdleMode.kBrake,
                kArm.EX_KP,
                kArm.EX_KI,
                kArm.EX_KD,
                kArm.EX_KF);

        m_leftMotorPid = m_leftArmMotor.getPIDController();

        //m_extenderMotor.getEncoder().setInverted(true);

        m_pot = new AnalogPotentiometer(DrivetrainConstants.ARM_POT_CHANNEL, 360, -(258.661793 - 180) / (90 / 44)); //-(350 - 90) / (90 / 44)
        m_exPot = new AnalogPotentiometer(DrivetrainConstants.EX_POT_CHANNEL, 100,-27.247387);//TODO: Change 

        // m_leftArmMotor.getEncoder().setPositionConversionFactor(
        //         360.0 / kArm.GEAR_RATIO); // degrees

        // m_leftArmMotor.getEncoder().setVelocityConversionFactor(
        //         (360.0 / kArm.GEAR_RATIO) / 60.0); // degrees per second

        //m_extenderMotor.getEncoder().setPositionConversionFactor(
        //        0.324 / kArm.GEAR_RATIO_EX); 

        // m_extenderMotor.getEncoder().setVelocityConversionFactor(
        //         (100.0 / kArm.GEAR_RATIO_EX) / 60.0); // degrees per second

        m_leftMotorPid.setOutputRange(kArm.MIN_PID_OUTPUT, kArm.MAX_PID_OUTPUT); 

        m_leftMotorPid.setSmartMotionMaxVelocity(kArm.SMART_MOTION_MAX_VELOCITY, 0);
        m_leftMotorPid.setSmartMotionMinOutputVelocity(kArm.SMART_MOTION_MIN_VELOCITY, 0);
        m_leftMotorPid.setSmartMotionMaxAccel(kArm.SMART_MOTION_MAX_ACCEL, 0);
        m_leftMotorPid.setSmartMotionAllowedClosedLoopError(0.13889, 0); //0.002
    
        m_rightArmMotor.follow(m_leftArmMotor, true);

        resetArm();

        
    }

    protected double getArmError(double target) {
        return target - getArmAbsolutePositionDegrees();
    }

    protected boolean isArmAtPos(double angle) {
        return Math.abs(getArmError(angle)) < kArm.ERROR;
    }

    protected double getExtError(double target) {
        return target - getArmAbsolutePositionDegrees();
    }

    protected boolean isExtAtPos(double angle) {
        return Math.abs(getExtError(angle)) < kArm.ERROR;
    }

    protected double getArmAbsolutePositionDegrees() {
        return (m_pot.get() * -(90.0 / (217.6 - 173.06)) + 360);
    }

    protected double getArmEncoderPositionDegrees() {
        return m_leftArmMotor.getEncoder().getPosition() * 360.0 / kArm.GEAR_RATIO - kArm.MIN_POSITION;
    }

    protected double degreesToArmEncoderRotations(double degrees) {
        return (degrees + kArm.MIN_POSITION) / 360.0 * kArm.GEAR_RATIO;
    }

    protected double getArmEncoderVelocityDegreesSec() {
        return m_leftArmMotor.getEncoder().getVelocity() * (360.0 / kArm.GEAR_RATIO) / 60.0;
    }

    protected double getExtenderAbsolutePosition() {
        return (m_exPot.get()); //* -(90.0 / (217.6 - 173.06)) + 360);
    }

    public void resetArm() {
        m_leftArmMotor.getEncoder().setPosition(degreesToArmEncoderRotations(getArmAbsolutePositionDegrees()));
    }

    public void resetExtender() {
        m_extenderMotor.getEncoder().setPosition(m_exPot.get());
    }

    // Main command to rotate and extend arm to a preset (angle and whether extended or not: enum ArmPos)
    public Command moveArm(ArmPos angle) {
        return runOnce(() -> {
            m_targetArmPosition = normalizeAngle(angle.getAngle());
        });
    }

    // A convinence function for moveArm method
    private void setPosition(double target, double armFF) {
        m_leftMotorPid.setReference(
                target,
                CANSparkMax.ControlType.kPosition,
                0,
                armFF);
                SmartDashboard.putNumber("reference target", target);
                SmartDashboard.putNumber("Computed FF", armFF);
    }

    // Sets a target for the arm to reach for the PID loop
    public void moveArm(double target, double velocity) {
        target = normalizeAngle(target);

        double armFF = m_armFeedforward.calculate(
                Units.degreesToRadians(target /*- kArm.FEEDFORWARD_ANGLE_OFFSET*/),velocity);
        System.out.println("ARMFF: " + armFF);
        System.out.println("PID TARGET ANGLE: " + target);
        setPosition(target, armFF);
        SmartDashboard.putNumber("Arm FeedForward", armFF);
    }

    //Tells the PID loop a target position to reach
    public void setExtenderPosition(double target, double extFF) {
        m_extenderMotor.getPIDController().setReference(
                target,
                CANSparkMax.ControlType.kPosition,
                0,
                extFF);
    }

    // Waits until the PID loop gets the extender to a set point
    public Command extend(boolean extended) {
        return runOnce(() -> {
            // m_extenderMotor.set(0.3);
            extenderTarget = kArm.EXTENDED_POSITION;
            //System.out.println("yes that");
        });
        // .until(() -> {
        //     var isDone = isExtAtPos(extended ? kArm.EXTENDED_POSITION : 0);
        //     SmartDashboard.putBoolean("ARM Extender at setpoint (command finished)", extended);
        //     return isDone;
        // });
                //.finallyDo(end -> m_extenderMotor.set(0));
    }

    public Command extendOpenLoop() {
        return run(() -> {
            m_extenderMotor.set(0.20);
        }).finallyDo(end -> m_extenderMotor.set(0));
    }

    // Gives you a max and min angle range for arm rotation
    public double normalizeAngle(double angle) {
        if (angle < kArm.MIN_POSITION) {
            return kArm.MIN_POSITION;
        } else if (angle > kArm.MAX_POSITION) {
            return kArm.MAX_POSITION;
        }
        return angle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm: Pot Position", m_pot.get());
        SmartDashboard.putNumber("Arm: Relative Encoder Pos (Raw)", m_leftArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm: Relative Encoder Pos (Adjusted Degrees)", getArmEncoderPositionDegrees());
        SmartDashboard.putNumber("Arm: Absolute Encoder Pos (pot position)", getArmAbsolutePositionDegrees());
        
        SmartDashboard.putNumber("Extender: Pot Position", m_exPot.get());
        SmartDashboard.putNumber("Extender: Relative Encoder Pos", m_extenderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extender: Absolute Encoder Pos (pot position)", getExtenderAbsolutePosition());

        SmartDashboard.putNumber("Arm: Setpoint position", (m_targetArmPosition != null) ? m_targetArmPosition : 0); 
        // SmartDashboard.putNumber("Arm: Setpoint velocity", m_currentSetpoint.velocity); 
        // if (RobotState.isEnabled()) {
            setExtenderPosition(extenderTarget, 0.15);
        // } else if (!RobotState.isEnabled()) {
        //     extenderTarget = m_extenderMotor.getEncoder().getPosition();
        // }
        // moveArm(m_currentSetpoint.position, m_currentSetpoint.velocity);

        // Resets encoder based off of pot values
        if (Math.abs(getArmAbsolutePositionDegrees() - getArmEncoderPositionDegrees()) > 1) {
             resetArm();
        }

        if (m_targetArmPosition != null) {
            m_targetArmPosition = normalizeAngle(m_targetArmPosition);

            // Calculate feed forward based on angle to counteract gravity
            // double cosineScalar = Math.cos(getWristPosition());
            // double feedForward = GRAVITY_FF * cosineScalar;
            double armFeedForward = m_armFeedforward.calculate(
                Units.degreesToRadians(getArmEncoderPositionDegrees()),
                getArmEncoderVelocityDegreesSec() * Math.PI / 180.0);

            SmartDashboard.putNumber("Arm: Computed feedforward", armFeedForward);

            m_leftMotorPid.setReference(degreesToArmEncoderRotations(m_targetArmPosition), 
                ControlType.kSmartMotion, 0, armFeedForward, ArbFFUnits.kVoltage);
          }
      
        // if (Math.abs(m_exPot.get() - m_extenderMotor.getEncoder().getPosition()) > 1) {
        //     resetExtender();
        // }
    }

}