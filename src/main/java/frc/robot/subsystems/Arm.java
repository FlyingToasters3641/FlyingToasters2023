package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.lib.util.MotorHelper;
import frc.lib.util.TunableNumber;

public class Arm extends SubsystemBase {
    private SparkMaxPIDController m_leftMotorPid;
    private CANSparkMax m_leftArmMotor;
    private CANSparkMax m_rightArmMotor;
    private CANSparkMax m_extenderMotor;

    private AnalogPotentiometer m_pot;
    private AnalogPotentiometer m_exPot;

    private ArmFeedforward m_armFeedforward;

    public static final class kArm {
        public static final double GEAR_RATIO = 90 / 1;
        public static final double KP = .01;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0;
        public static final double KS = 0.11814;
        public static final double KG = 0.49202;
        public static final double KV = 0.034012;
        public static final double KA = 0.0020898;
        public static final int LEFT_CURRENT_LIMIT = 25;
        public static final int RIGHT_CURRENT_LIMIT = 25;

        // values for Extender
        public static final double GEAR_RATIO_EX = 9 / 1;
        public static final double EX_KP = 0.01;
        public static final double EX_KI = 0.0;
        public static final double EX_KD = 0.0;
        public static final double EX_KF = 0.0;
        public static final double EXTENDED_POSITION = 0.076; // TODO: measure analog pot for extender.

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

        m_leftArmMotor.getEncoder().setPositionConversionFactor(
                360.0 / kArm.GEAR_RATIO); // degrees

        m_leftArmMotor.getEncoder().setVelocityConversionFactor(
                (360.0 / kArm.GEAR_RATIO) / 60.0); // degrees per second

         m_extenderMotor.getEncoder().setPositionConversionFactor(
                0.324 / kArm.GEAR_RATIO_EX); 

        // m_extenderMotor.getEncoder().setVelocityConversionFactor(
        //         (100.0 / kArm.GEAR_RATIO_EX) / 60.0); // degrees per second

        m_rightArmMotor.follow(m_leftArmMotor, true);

        resetArm();
    }

    protected double getArmError(double target) {
        return target - getArmAbsolutePosition();
    }

    protected boolean isArmAtPos(double angle) {
        return Math.abs(getArmError(angle)) < kArm.ERROR;
    }

    protected double getExtError(double target) {
        return target - getArmAbsolutePosition();
    }

    protected boolean isExtAtPos(double angle) {
        return Math.abs(getExtError(angle)) < kArm.ERROR;
    }

    protected double getArmAbsolutePosition() {
        return (m_pot.get() * -(90.0 / (217.6 - 173.06)) + 360);
    }

    protected double getExtenderAbsolutePosition() {
        return (m_exPot.get()); //* -(90.0 / (217.6 - 173.06)) + 360);
    }

    public void resetArm() {
        m_leftArmMotor.getEncoder().setPosition(getArmAbsolutePosition());
    }

    // public void resetExtender() {
    //     m_extenderMotor.getEncoder().setPosition(m_exPot.get());
    // }

    // Main command to rotate and extend arm to a preset (angle and whether extended or not: enum ArmPos)
    TrapezoidProfile.State m_currentSetpoint;
    public Command moveArm(ArmPos angle) {
        m_currentSetpoint = new TrapezoidProfile.State(getArmAbsolutePosition(),0); //current state
        var endgoal = new TrapezoidProfile.State(angle.getAngle(), 0); //end goal
        double startTime = Timer.getFPGATimestamp();

        resetArm();
        return run(() -> {
            TrapezoidProfile armProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3, 0.5),
                    endgoal, m_currentSetpoint);
            m_currentSetpoint = armProfile.calculate(0.02);

            System.out.println("Set point position" + m_currentSetpoint.position); 
            System.out.println("Set point velocity" + m_currentSetpoint.velocity); 
            System.out.println("end goal position" + endgoal.position); 

            moveArm(m_currentSetpoint.position, m_currentSetpoint.velocity);
            System.out.println("TARGET ANGLE (in command): " + angle.getAngle());

            // moveArm(angle.getAngle(), 0);
        }).finallyDo(end -> m_leftArmMotor.set(0)); //.until(() -> isArmAtPos(angle.getAngle()))
         //       .finallyDo(end -> m_leftArmMotor.set(0));
        // .andThen(() -> extend(angle.getExtended()));
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
        return run(() -> {
            // m_extenderMotor.set(0.3);
            setExtenderPosition(extended ? kArm.EXTENDED_POSITION : 0, .1);
            System.out.println("yes that");
        }).until(() -> isExtAtPos(extended ? kArm.EXTENDED_POSITION : 0))
                .finallyDo(end -> m_extenderMotor.set(0));
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
        SmartDashboard.putNumber("Arm: Relative Encoder Pos", m_leftArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm: Absolute Encoder Pos (pot position)", getArmAbsolutePosition());
        
        SmartDashboard.putNumber("Extender: Pot Position", m_exPot.get());
        SmartDashboard.putNumber("Extender: Relative Encoder Pos", m_extenderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extender: Absolute Encoder Pos (pot position)", getExtenderAbsolutePosition());

        // Resets enconder based off of pot values
         if (Math.abs(getArmAbsolutePosition() - m_leftArmMotor.getEncoder().getPosition()) > 1) {
             resetArm();
        }
        // if (Math.abs(m_exPot.get() - m_extenderMotor.getEncoder().getPosition()) > 1) {
        //     resetExtender();
        // }
    }

}