package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MotorHelper;
import frc.robot.Constants.DrivetrainConstants;

public class IntakeEffector extends SubsystemBase {

    private final DoubleSolenoid solenoid;
    int intakeFilterIteration = 20;

    private CANSparkMax m_rollers;
    // private DutyCycleEncoder m_tof;
    private DutyCycle m_tof;
    private LinearFilter m_tofFilter;

    private boolean intakeRetracted;
    SparkMaxPIDController PIDController;
    double prevEncoder;

    public IntakeEffector() {
        solenoid =
                new DoubleSolenoid(
                        DrivetrainConstants.PNEUMATIC_HUB,
                        PneumaticsModuleType.REVPH,
                        DrivetrainConstants.EFFECTOR_EXTENDED_CHANNEL,
                        DrivetrainConstants.EFFECTOR_RETRACTED_CHANNEL
                );

        solenoid.set(Value.kReverse);
        intakeRetracted = true;

        m_rollers =
                MotorHelper.createSparkMax(
                        DrivetrainConstants.ROLLER_MOTOR,
                        MotorType.kBrushless,
                        false,
                        DrivetrainConstants.ROLLER_MOTOR_LIMIT,
                        IdleMode.kBrake
                );
        PIDController = m_rollers.getPIDController();
        PIDController.setP(0.05);
        PIDController.setI(0);
        PIDController.setD(0);

        prevEncoder = m_rollers.getEncoder().getPosition();

        m_tof = new DutyCycle(new DigitalInput(0));
        //m_tof.setDistancePerRotation(1.0);
        // m_tof.reset();
        m_tofFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    }

    public boolean isIn() {
        return intakeRetracted;
    }

    private double avgCurrent = 0;
    private double currentStore = 0;
    private int intakeIteration = 1;
    // private double tofPos = m_tof.getAbsolutePosition();

    // public boolean intakeHasObject() {
    //     if (tofPos > 0.3 || tofPos < 0.15) 
    //         return true;
    //     else
    //         return false;
    // }

    public Command runIntake(LEDSubsystem m_leds) {

        return run(() -> {


            if (intakeIteration < intakeFilterIteration) {
                currentStore += m_rollers.getOutputCurrent();
                intakeIteration++;
            } else if (intakeIteration == intakeFilterIteration) {
                avgCurrent = currentStore / intakeIteration;
                intakeIteration++;
            } else {
                intakeIteration = 1;
                currentStore = 0;
            }

            SmartDashboard.putNumber("average current", avgCurrent);
            m_rollers.set(0.75);

            SmartDashboard.putBoolean("Intake is finished", false);
        })
                .until(() -> {
                    return avgCurrent >= 20;
                })
                .andThen(() -> {
                    m_rollers.set(0);
                    m_leds.ledSwitch(5);  // flash green when we have an object
                    PIDController.setReference(
                            m_rollers.getEncoder().getPosition() * 1.15,
                            ControlType.kPosition
                    );
                    avgCurrent = 0;
                    currentStore = 0;
                    intakeIteration = 1;
                    SmartDashboard.putBoolean("Intake is finished", true);

                }).finallyDo(end -> {
                   
                        m_rollers.set(0);
                        PIDController.setReference(
                                m_rollers.getEncoder().getPosition(),
                                ControlType.kPosition
                        );
                    
                });
        // TODO: MAP TO A BUTTON FOR RUNNING THE ROLLERS
    }

    public Command extendIntake() {
        return runOnce(() -> {
            if (solenoid.get() == Value.kReverse) {
                toggleIntake();
            }
        });
        // TODO: MAP TO BUTTON FOR EXTENDING THE WRIST
    }

    public Command stopIntake() {
        return runOnce(() -> {
            m_rollers.set(0);
            PIDController.setReference(
                    m_rollers.getEncoder().getPosition(),
                    ControlType.kPosition
            );
        });
        // TODO: MAP TO BUTTON FOR STOPPING THE INTAKE
    }

    public Command retractIntake() {
        return runOnce(() -> {
            if (solenoid.get() == Value.kForward) {
                toggleIntake();
            }
        });
        // TODO: MAP TO BUTTON FOR RETRACTING THE WRIST
    }

    public Command reverseIntake() {
        return Commands.run(() -> {
            m_rollers.set(-0.5);
        })
                .finallyDo(end -> {
                    m_rollers.set(0);
                    PIDController.setReference(
                            m_rollers.getEncoder().getPosition(),
                            ControlType.kPosition
                    );
                });
        // TODO: MAP TO BUTTON FOR REVERSING THE INTAKE
    }

    public Command shootPiece() {
        return Commands.run(() -> {
            m_rollers.set(-1);
        })
                .finallyDo(end -> {
                    m_rollers.set(0);
                    PIDController.setReference(
                            m_rollers.getEncoder().getPosition(),
                            ControlType.kPosition
                    );
                });
        // TODO: MAP TO BUTTON FOR REVERSING THE INTAKE
    }

    private void toggleIntake() {
        solenoid.toggle();
        intakeRetracted = !intakeRetracted;
        // TODO: DO WE NEED THIS?
    }

    @Override
    public void periodic() {
        var currValue = m_tofFilter.calculate(avgCurrent);

        SmartDashboard.putNumber("ROLLER MOTOR TEMP", m_rollers.getMotorTemperature());

        SmartDashboard.putNumber(
                "Roller output current",
                m_rollers.getOutputCurrent()
        );
        SmartDashboard.putNumber(
                "Roller encoder",
                m_rollers.getEncoder().getPosition() - prevEncoder
        );
        SmartDashboard.putNumber(
                "Roller current / encoder",
                m_rollers.getOutputCurrent() /
                        (m_rollers.getEncoder().getPosition() - prevEncoder)
        );

        SmartDashboard.putNumber("TOF: Output", m_tof.getOutput());
        SmartDashboard.putNumber("TOF: Filtered output", currValue);
        SmartDashboard.putNumber("TOF: Freqency", m_tof.getFrequency());
        SmartDashboard.putNumber("TOF: HighTimeNanoseconds", m_tof.getHighTimeNanoseconds());

        prevEncoder = m_rollers.getEncoder().getPosition();
    }
}
