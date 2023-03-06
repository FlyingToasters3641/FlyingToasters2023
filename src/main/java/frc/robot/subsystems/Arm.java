package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.lib.util.MotorHelper;
import frc.lib.util.TunableNumber;

public class Arm extends SubsystemBase {
    public static final boolean TUNING_MODE = false;
    protected final TunableNumber armP;
    protected final TunableNumber armI;
    protected final TunableNumber armD;
    protected final TunableNumber armF;
    protected final TunableNumber targetPos;

    private SparkMaxPIDController m_leftMotorPid;
    private CANSparkMax m_leftArmMotor;
    private CANSparkMax m_rightArmMotor;

    private AnalogPotentiometer m_pot;

    private ArmFeedforward m_armFeedforward;

    public static final class kArm {
        public static final double GEAR_RATIO = 90 / 1;
        public static final double KP = 0.01;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0;
        public static final double KS = 0.32;
        public static final double KG = 0.42;
        public static final double KV = 0.01826;
        public static final double KA = 0.0019367;
        public static final int LEFT_CURRENT_LIMIT = 25;
        public static final int RIGHT_CURRENT_LIMIT = 25;

        public static final double ERROR = 5.0; // degrees
        public static final double MAX_POSITION = 210;  // degrees

        public static final double FEEDFORWARD_ANGLE_OFFSET = 30; //TODO: find out what this offset is. 
    }

    public Arm() {
        m_rightArmMotor.setInverted(true);

        m_leftArmMotor.setSmartCurrentLimit(25);
        m_rightArmMotor.setSmartCurrentLimit(25);
        m_leftArmMotor.setIdleMode(IdleMode.kBrake);
        m_rightArmMotor.setIdleMode(IdleMode.kBrake);

        m_rightArmMotor.follow(m_leftArmMotor);

        m_armFeedforward = new ArmFeedforward(
            kArm.KS,
            kArm.KG,
            kArm.KV,
            kArm.KA
        );

        // m_pot.get();

        armP = new TunableNumber("Arm P", kArm.KP, TUNING_MODE);
        armI = new TunableNumber("Arm I", kArm.KI, TUNING_MODE);
        armD = new TunableNumber("Arm D", kArm.KD, TUNING_MODE);
        armF = new TunableNumber("Arm F", kArm.KF, TUNING_MODE);
        targetPos = new TunableNumber(
                "Target Pos",
                ArmPos.LOWERED.getAngle(),
                TUNING_MODE);

        m_leftArmMotor = MotorHelper.createSparkMax(
                DrivetrainConstants.LEFT_ARM_MOTOR,
                MotorType.kBrushless,
                false,
                kArm.LEFT_CURRENT_LIMIT,
                IdleMode.kBrake,
                armP.get(),
                armI.get(),
                armD.get(),
                armF.get());

        m_rightArmMotor = MotorHelper.createSparkMax(
                DrivetrainConstants.RIGHT_ARM_MOTOR,
                MotorType.kBrushless,
                true,
                kArm.RIGHT_CURRENT_LIMIT,
                IdleMode.kBrake,
                armP.get(),
                armI.get(),
                armD.get(),
                armF.get());

        m_leftMotorPid = m_leftArmMotor.getPIDController();

        // Initializes an AnalogPotentiometer on analog port 0
        // The full range of motion (in meaningful external units) is 0-180 (this could be degrees, for instance)
        // The "starting point" of the motion, i.e. where the mechanism is located when the potentiometer reads 0v, is 30.
        m_pot = new AnalogPotentiometer(DrivetrainConstants.ARM_POT_CHANNEL, 270, 30);

        m_leftArmMotor.getEncoder().setPositionConversionFactor(
                360.0 / kArm.GEAR_RATIO); // degrees

        m_leftArmMotor.getEncoder().setVelocityConversionFactor(
                (360.0 / kArm.GEAR_RATIO) / 60.0); // degrees per second

        m_rightArmMotor.follow(m_leftArmMotor, true);

        resetArm();
    }

    protected void update() {
        if (armP.hasChanged()) {
            m_leftMotorPid.setP(armP.get());
        }
        if (armI.hasChanged()) {
            m_leftMotorPid.setI(armI.get());
        }
        if (armD.hasChanged()) {
            m_leftMotorPid.setD(armD.get());
        }
        if (armF.hasChanged()) {
            m_leftMotorPid.setFF(armF.get());
        }

        if (targetPos.hasChanged()) {
            moveArm(targetPos.get());
        }
    }

    protected double getError(double target) {
        return target - getAbsolutePosition();
    }

    protected boolean isAtPos(double angle) {
        return Math.abs(getError(angle)) < kArm.ERROR;
    }

    protected double getAbsolutePosition() {
        return m_pot.get();
    }

    public void setPosition(double target, double armFF) {
        targetPos.setDefault(target);

        m_leftMotorPid.setReference(
                target,
                CANSparkMax.ControlType.kPosition,
                0,
                armFF);
    }

    public void moveArm(double target) {
        target = normalizeAngle(target);

        double armFF = m_armFeedforward.calculate(
                Units.degreesToRadians(target - kArm.FEEDFORWARD_ANGLE_OFFSET),
                0);
        setPosition(target, armFF);
        SmartDashboard.putNumber("Arm FeedForward", armFF);
    }

    public Command moveArm(ArmPos angle) {
        return run(() -> {
            moveArm(angle.getAngle());
        }).until(() -> isAtPos(angle.getAngle()));
    }

    public void resetArm() {
        m_leftArmMotor.getEncoder().setPosition(getAbsolutePosition());
    }

    public double normalizeAngle(double angle) {
        if (angle < 0) {
            return 0;
        } else if (angle > kArm.MAX_POSITION) {
            return kArm.MAX_POSITION;
        }
        return angle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Pot: Position", m_pot.get());
        SmartDashboard.putNumber("Left Arm Motor Encoder: Position", m_leftArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Pot: Position", getAbsolutePosition());
        SmartDashboard.putNumber("Relative Encoder Pos", m_leftArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Absolute Encoder Pos", getAbsolutePosition());
        SmartDashboard.putNumber("Arm Target Pose", targetPos.get());
        if (Math.abs(getAbsolutePosition() - m_leftArmMotor.getEncoder().getPosition()) > 1) {
            resetArm();
        }
    }

}