package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakePos;
import frc.lib.util.MotorHelper;

public class Arm extends SubsystemBase {
    private SparkMaxPIDController m_leftMotorPid;
    private SparkMaxPIDController m_extenderPid;
    private CANSparkMax m_leftArmMotor;
    private CANSparkMax m_rightArmMotor;
    private CANSparkMax m_extenderMotor;
    private AnalogPotentiometer m_pot;
    private AnalogPotentiometer m_exPot;
    private ArmFeedforward m_armFeedforward;

    private IntakeEffector m_intake;

    double m_extenderTarget;
    private Double m_targetArmPosition = null;
    private ArmPos m_targetArmState = null;

    public static final class kArm {
        public static final double GEAR_RATIO = 90 / 1;
        public static final double KP = 0.00009; // 0.0025; //0.04;//0.090071;//0.0001;
        public static final double KI = 0;// 0.0001;
        public static final double KD = 0;// 0.000015; //0.017546;//0.0;
        public static final double KF = 0.0;
        public static final double KS = 0.078474;// 0.11814;
        public static final double KG = 0.53836;// 0.49202;
        public static final double KV = 0.032529;// 0.034012;
        public static final double KA = 0.00078541;// 0.0020898;
        public static final int LEFT_CURRENT_LIMIT = 39;
        public static final int RIGHT_CURRENT_LIMIT = 39;
        public static final double MIN_PID_OUTPUT = -0.5;
        public static final double MAX_PID_OUTPUT = 0.5;
        public static final double SMART_MOTION_MAX_VELOCITY = 9000; // rpm
        public static final double SMART_MOTION_MAX_ACCEL = 14000;
        public static final double SMART_MOTION_MIN_VELOCITY = 0; // rpm

        // values for Extender
        public static final double GEAR_RATIO_EX = 9 / 1;
        public static final double EX_KP = 0.000070; // 0.025;// 0.015;
        public static final double EX_KI = 0.0;
        public static final double EX_KD = 0.0000004; // 0.0001;
        public static final double EX_KF = 0.0005;
        public static final double EX_KG = 0.4;
        public static final double EXTENDED_POSITION = 90;// 26.23; // TODO: measure analog pot for extender.

        public static final double ERROR = 5.0; // degrees
        public static final double MIN_POSITION = -59.0; // degrees
        public static final double MAX_POSITION = 180; // degrees
    }

    public Arm(IntakeEffector intake) {
        m_intake = intake;

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
                kArm.RIGHT_CURRENT_LIMIT,
                IdleMode.kBrake,
                kArm.EX_KP,
                kArm.EX_KI,
                kArm.EX_KD,
                kArm.EX_KF);

        m_leftMotorPid = m_leftArmMotor.getPIDController();
        m_extenderPid = m_extenderMotor.getPIDController();

        m_pot = new AnalogPotentiometer(DrivetrainConstants.ARM_POT_CHANNEL, 360, -(258.661793 - 180) / (90 / 44));
        m_exPot = new AnalogPotentiometer(DrivetrainConstants.EX_POT_CHANNEL, 100, -27.247387);

        //m_leftMotorPid.setOutputRange(kArm.MIN_PID_OUTPUT, kArm.MAX_PID_OUTPUT);
        m_leftMotorPid.setSmartMotionMaxVelocity(kArm.SMART_MOTION_MAX_VELOCITY, 0);
        m_leftMotorPid.setSmartMotionMinOutputVelocity(kArm.SMART_MOTION_MIN_VELOCITY, 0);
        m_leftMotorPid.setSmartMotionMaxAccel(kArm.SMART_MOTION_MAX_ACCEL, 0);
        m_leftMotorPid.setSmartMotionAllowedClosedLoopError(0.13889, 0); // 0.002

        m_extenderPid.setOutputRange(-0.5, 0.5);
        m_extenderPid.setSmartMotionMaxVelocity(2000/* 1500 */, 0);
        m_extenderPid.setSmartMotionMaxAccel(1500/* 1000 */, 0);
        m_extenderPid.setSmartMotionAllowedClosedLoopError(1.0, 0); // 0.002

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
        return target - getExtenderAbsolutePosition();
    }

    protected boolean isExtAtPos(double pos) {
        return Math.abs(getExtError(pos)) < 2;
    }

    public double getArmAbsolutePositionDegrees() {
        return (m_pot.get() * -(90.0 / (217.6 - 173.06)) + 360);
    }

    protected double getArmEncoderPositionDegrees() {
        return m_leftArmMotor.getEncoder().getPosition() * 360.0 / kArm.GEAR_RATIO - kArm.MIN_POSITION;
    }

    protected double getExtenderEncoderPosition() {
        return ((m_extenderMotor.getEncoder().getPosition() - 0.0) * 2.30898432171);// - 2.520537;
    }

    protected double degreesToArmEncoderRotations(double degrees) {
        return (degrees + kArm.MIN_POSITION) / 360.0 * kArm.GEAR_RATIO;
    }

    protected double getArmEncoderVelocityDegreesSec() {
        return m_leftArmMotor.getEncoder().getVelocity() * (360.0 / kArm.GEAR_RATIO) / 60.0;
    }

    protected double getExtenderAbsolutePosition() {
        return (((m_exPot.get() - 0.114723) * 2.259920049) * 1.02526223259);// - 2.208095;
    }

    public void resetArm() {
        m_leftArmMotor.getEncoder().setPosition(degreesToArmEncoderRotations(getArmAbsolutePositionDegrees()));
    }

    public void resetExtender() {
        m_extenderMotor.getEncoder()
                .setPosition(((getExtenderAbsolutePosition() / 2.259920049) / 1.02526223259) + 0.114723);
    }

    // Main command to rotate and extend arm to a preset (angle and whether extended
    // or not: enum ArmPos)
    public Command moveArm(ArmPos angle) {
        // return extend(0)  // always retract elevator before rotating arm
        // //.andThen(() -> m_intake.retractIntake())  // always make sure in "Default" position before rotating
        // .andThen(run(() -> m_targetArmPosition = normalizeAngle(angle.getAngle()))
        // .until(() -> {
        //     var done = isArmAtPos(angle.getAngle());
        //     System.out.println("Done: " + done + ", Target: " + angle.getAngle() + ", Current Angle: " + getArmAbsolutePositionDegrees());
        //     return done;
        // })
        // //.andThen(angle.getIntakePosition() == IntakePos.DEFAULT ? m_intake.retractIntake() : m_intake.extendIntake())
        // .andThen(() -> extend(angle.getExtended())));

        return extend(0).andThen(
            run(() -> m_targetArmPosition = normalizeAngle(angle.getAngle()))
            .until(() -> isArmAtPos(angle.getAngle())).withTimeout(7)
                .andThen((angle.getIntakePosition() == IntakePos.DEFAULT ? m_intake.retractIntake() : m_intake.extendIntake())
                    .andThen(extend(angle.getExtended())))
        );
    
    }

    public Command extend(double position) {
        return run(() -> {
            m_extenderTarget = position;
        }).until(() -> isExtAtPos(position)).withTimeout(.5);
        //.withTimeout(0.5); // don't wait forever - assume its close and bail
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
                Units.degreesToRadians(target), velocity);
        setPosition(target, armFF);
    }

    // Tells the PID loop a target position to reach
    public void setExtenderPosition(double target, double extFF) {
        m_extenderMotor.getPIDController().setReference(
                (((target / 2.259920049) / 1.02526223259) + 0.114723),
                CANSparkMax.ControlType.kSmartMotion,
                0,
                extFF,
                ArbFFUnits.kVoltage);
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

        //SmartDashboard.putNumber("Extender: Pot Position", m_exPot.get());
        SmartDashboard.putNumber("Extender: Relative Encoder Pos", getExtenderEncoderPosition());
        SmartDashboard.putNumber("Extender: Absolute Encoder Pos", getExtenderAbsolutePosition());

        SmartDashboard.putNumber("Arm: Setpoint position", (m_targetArmPosition != null) ? m_targetArmPosition : 0);
        // SmartDashboard.putNumber("Arm: Setpoint velocity",
        // m_currentSetpoint.velocity);

        // Resets encoder based off of pot values
        if (Math.abs(getArmAbsolutePositionDegrees() - getArmEncoderPositionDegrees()) > 1) {
            resetArm();
        }
        if (Math.abs(getExtenderAbsolutePosition() - getExtenderEncoderPosition()) > 0.25) {
            resetExtender();
        }

        if (!RobotState.isEnabled()) {
            m_targetArmPosition = getArmAbsolutePositionDegrees();
            m_extenderTarget = 0;
        }

        // Maintain extender setpoint and compute feedforward.
        //  Compute a feedforward based on the arm's rotation angle.
        //   We only want to add a feedforward if the elevator needs help against gravity.
        var armAngleDegrees = getArmEncoderPositionDegrees();
        double extenderFeedForward = kArm.EX_KG * Math.sin(Units.degreesToRadians(armAngleDegrees));
        if ((m_extenderTarget == 0.0 && Math.signum(extenderFeedForward) == 1.0) ||
            (m_extenderTarget > 0.0 && Math.signum(extenderFeedForward) == -1.0)) {
                extenderFeedForward = 0.0;  // don't fight gravity if it's helping us
        }
        setExtenderPosition(m_extenderTarget, extenderFeedForward);

        // Maintain arm setpoint and compute feedforward
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

        SmartDashboard.putNumber("Extender target", m_extenderTarget);
    }

}