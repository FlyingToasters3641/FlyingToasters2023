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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.lib.util.MotorHelper;
import frc.lib.util.TunableNumber;

public class Arm extends SubsystemBase {
    // public static final boolean TUNING_MODE = false;
    // protected final TunableNumber armP;
    // protected final TunableNumber armI;
    // protected final TunableNumber armD;
    // protected final TunableNumber armF;
    // protected final TunableNumber targetPos;//HUH

    // Pid for extender
    // protected final TunableNumber exP;
    // protected final TunableNumber exI;
    // protected final TunableNumber exD;
    // protected final TunableNumber exF;

    private SparkMaxPIDController m_leftMotorPid;
    private CANSparkMax m_leftArmMotor;
    private CANSparkMax m_rightArmMotor;
    // private CANSparkMax m_extenderMotor;

    private AnalogPotentiometer m_pot;
    // private AnalogPotentiometer m_exPot;

    private ArmFeedforward m_armFeedforward;

    public static final class kArm {
        public static final double GEAR_RATIO = 90 / 1;
        public static final double KP = .01;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0;
        public static final double KS = 0.11814;
        public static final double KG = 0.49202;//0.88645 //0.42
        public static final double KV = 0.034012;
        public static final double KA = 0.0020898;
        public static final int LEFT_CURRENT_LIMIT = 25;
        public static final int RIGHT_CURRENT_LIMIT = 25;

        // values for Extender
        // public static final double GEAR_RATIO_EX = 90 / 1;
        // public static final double EX_KP = 0.01;
        // public static final double EX_KI = 0.0;
        // public static final double EX_KD = 0.0;
        // public static final double EX_KF = 0.0;
        // // public static final double EX_KS = 0.32;
        // // public static final double EX_KG = 0.42;
        // // public static final double EX_KV = 0.01826;
        // // public static final double EX_KA = 0.0019367;
        // public static final double EXTENDED_POSITION = 0; // TODO: measure analog pot for extender.

        public static final double ERROR = 5.0; // degrees
        public static final double MIN_POSITION = -59.0; // degrees
        public static final double MAX_POSITION = 180; // degrees

        public static final double FEEDFORWARD_ANGLE_OFFSET = 30; // TODO: find out what this offset is.
    }

    public Arm() {
        m_armFeedforward = new ArmFeedforward(
                kArm.KS,
                kArm.KG,
                kArm.KV,
                kArm.KA);

        // m_pot.get();

        // armP = new TunableNumber("Arm P", kArm.KP, TUNING_MODE);
        // armI = new TunableNumber("Arm I", kArm.KI, TUNING_MODE);
        // armD = new TunableNumber("Arm D", kArm.KD, TUNING_MODE);
        // armF = new TunableNumber("Arm F", kArm.KF, TUNING_MODE);

        // // For extender
        // exP = new TunableNumber("Extender P", kArm.EX_KP, TUNING_MODE);
        // exI = new TunableNumber("Extender I", kArm.EX_KI, TUNING_MODE);
        // exD = new TunableNumber("Extender D", kArm.EX_KD, TUNING_MODE);
        // exF = new TunableNumber("Extender F", kArm.EX_KF, TUNING_MODE);

        // targetPos = new TunableNumber(
        //         "Target Pos",
        //         ArmPos.STORED_POSITION.getAngle(),
        //         TUNING_MODE);//HUH

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

        // m_extenderMotor = MotorHelper.createSparkMax(
        //         DrivetrainConstants.EXTENDER_MOTOR,
        //         MotorType.kBrushless,
        //         false,
        //         kArm.RIGHT_CURRENT_LIMIT, // TODO: verify current limit
        //         IdleMode.kBrake,
        //         exP.get(),
        //         exI.get(),
        //         exD.get(),
        //         exF.get());

        m_leftMotorPid = m_leftArmMotor.getPIDController();

        //m_rightArmMotor.setInverted(true);

        //m_leftArmMotor.setSmartCurrentLimit(25);
        //m_rightArmMotor.setSmartCurrentLimit(25);
        //m_leftArmMotor.setIdleMode(IdleMode.kBrake);
        //m_rightArmMotor.setIdleMode(IdleMode.kBrake);

        m_pot = new AnalogPotentiometer(DrivetrainConstants.ARM_POT_CHANNEL, 360, -(258.661793 - 180) / (90 / 44)); //-(350 - 90) / (90 / 44)
        // m_exPot = new AnalogPotentiometer(DrivetrainConstants.EX_POT_CHANNEL, 270, 30);

        m_leftArmMotor.getEncoder().setPositionConversionFactor(
                360.0 / kArm.GEAR_RATIO); // degrees

        m_leftArmMotor.getEncoder().setVelocityConversionFactor(
                (360.0 / kArm.GEAR_RATIO) / 60.0); // degrees per second

        //m_extenderMotor.getEncoder().setPositionConversionFactor(
        //        360.0 / kArm.GEAR_RATIO); // TODO: change gear ratio to 9/1

        m_rightArmMotor.follow(m_leftArmMotor, true);

        resetArm();
    }

    // Gets updates from Smart Dashboard on PID values and sets them on the motor
    protected void update() {
        // if (armP.hasChanged()) {
        //     m_leftMotorPid.setP(armP.get());
        // }
        // if (armI.hasChanged()) {
        //     m_leftMotorPid.setI(armI.get());
        // }
        // if (armD.hasChanged()) {
        //     m_leftMotorPid.setD(armD.get());
        // }
        // if (armF.hasChanged()) {
        //     m_leftMotorPid.setFF(armF.get());
        // }

        // if (exP.hasChanged()) {
        //     m_extenderMotor.getPIDController().setP(exP.get());
        // }
        // if (exI.hasChanged()) {
        //     m_extenderMotor.getPIDController().setI(exI.get());
        // }
        // if (exD.hasChanged()) {
        //     m_extenderMotor.getPIDController().setD(exD.get());
        // }
        // if (exF.hasChanged()) {
        //     m_extenderMotor.getPIDController().setFF(exF.get());
        // }

        // if (targetPos.hasChanged()) {
        //     moveArm(targetPos.get(), 0);
        // }
    }

    protected double getArmError(double target) {
        return target - getAbsolutePosition();
    }//NEGATIVE VALUE TO THE 0 ANGLE

    protected boolean isArmAtPos(double angle) {
        return Math.abs(getArmError(angle)) < kArm.ERROR;
    }//BOOLEAN IS ARM ERROR GREATER THAN 5 DEGREES

    // protected double getExtError(double target) {
    //     return target - getAbsolutePosition();
    // }

    // protected boolean isExtAtPos(double angle) {
    //     return Math.abs(getExtError(angle)) < kArm.ERROR;
    // }

    protected double getAbsolutePosition() {
        return (m_pot.get() * -(90.0 / (217.6 - 173.06)) + 360);
    }

    public void resetArm() {
        m_leftArmMotor.getEncoder().setPosition(getAbsolutePosition());
    }

    // public void resetExtender() {
    //     m_extenderMotor.getEncoder().setPosition(m_exPot.get());
    // }

    // Main command to rotate and extend arm to a preset (angle and whether extended
    // or not: enum ArmPos)
    public Command moveArm(ArmPos angle) {
        // var currentSetpoint = new TrapezoidProfile.State(0,0); //current state
        // var endgoal = new TrapezoidProfile.State(angle.getAngle(), 0); //end goal
        resetArm();
        return run(() -> {
            // TrapezoidProfile armProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
            //         endgoal, currentSetpoint);
            // var setpoint = armProfile.calculate(.02);
            // currentSetpoint.position = setpoint.position;
            // currentSetpoint.velocity = setpoint.velocity;

            // System.out.println("Set point position" + setpoint.position); 
            // System.out.println("Set point velocity" + setpoint.velocity); 
            // System.out.println("end goal position" + endgoal.position); 

            // moveArm(currentSetpoint.position, currentSetpoint.velocity);
            System.out.println("TARGET ANGLE (in command): " + angle.getAngle());
            moveArm(angle.getAngle(), 0);
        }).finallyDo(end -> m_leftArmMotor.set(0)); //.until(() -> isArmAtPos(angle.getAngle()))
         //       .finallyDo(end -> m_leftArmMotor.set(0));
        // .andThen(() -> extend(angle.getExtended()));
    }

    // A convinence function for moveArm method
    private void setPosition(double target, double armFF) {
        // targetPos.setDefault(target);

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

    // Tells the PID loop a target position to reach
    // public void setExtenderPosition(double target, double extFF) {
    //     targetPos.setDefault(target);
    //     m_extenderMotor.getPIDController().setReference(
    //             target,
    //             CANSparkMax.ControlType.kPosition,
    //             0,
    //             extFF);
    // }

    // // Waits until the PID loop gets the extender to a set point
    // public Command extend(boolean extended) {
    //     return run(() -> {
    //         setExtenderPosition(extended ? kArm.EXTENDED_POSITION : 0, 0);
    //     }).until(() -> isExtAtPos(extended ? kArm.EXTENDED_POSITION : 0))
    //             .finallyDo(end -> m_extenderMotor.set(0));
    // }

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
        SmartDashboard.putNumber("Arm: Absolute Encoder Pos (pot position)", getAbsolutePosition());

        System.out.println("LEFT ARM APPLIED VOLTAGE: " + m_leftArmMotor.getAppliedOutput());
        System.out.println("RIGHT ARM APPLIED VOLTAGE: " + m_rightArmMotor.getAppliedOutput());
        // SmartDashboard.putNumber("Arm: Target Pose", targetPos.get());
        // SmartDashboard.putNumber("Extender: Pot Value", m_exPot.get());
        // SmartDashboard.putNumber("Extender: Relative Encoder Pos", m_extenderMotor.getEncoder().getPosition());

        /*
         * BACKUP IF SMARTDASHBOARD DOES NOT WORK
         * System.out.println (String.format("Arm: Relative Encoder Pos = %f",
         * m_leftArmMotor.getEncoder().getPosition()));
         * System.out.println
         * (String.format("Arm: Absolute Encoder Pos (pot position) = %f",
         * getAbsolutePosition()));
         * System.out.println (String.format("Arm: Target Pose = %f", targetPos.get()));
         * System.out.println (String.format("Extender: Pot Value = %f",
         * m_exPot.get()));
         * System.out.println (String.format("Extender: Relative Encoder Pos = %f",
         * m_extenderMotor.getEncoder().getPosition()));
         */

        // Resets enconder based off of pot values
         if (Math.abs(getAbsolutePosition() - m_leftArmMotor.getEncoder().getPosition()) > 1) {
             resetArm();
        }
        // if (Math.abs(m_exPot.get() - m_extenderMotor.getEncoder().getPosition()) > 1) {
        //     resetExtender();
        // }
    }

}