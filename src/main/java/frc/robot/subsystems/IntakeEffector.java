package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MotorHelper;
import frc.robot.Constants.DrivetrainConstants;

public class IntakeEffector extends SubsystemBase {

    private final DoubleSolenoid solenoid;
    int intakeFilterIteration = 5;

    private CANSparkMax m_rollers;
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
    }

    public boolean isIn() {
        return intakeRetracted;
    }

    private double avgCurrent = 0;
    private double currentStore = 0;
    private int intakeIteration = 1;

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
                    return avgCurrent >= 15;
                })
                .andThen(() -> {
                    m_rollers.set(0);
                    //TODO: Add led stuff here
                    PIDController.setReference(
                            m_rollers.getEncoder().getPosition(),
                            ControlType.kPosition
                    );
                    avgCurrent = 0;
                    currentStore = 0;
                    intakeIteration = 1;
                    SmartDashboard.putBoolean("Intake is finished", true);

                }).finallyDo(end -> {
                    if (!end) {
                        m_rollers.set(0);
                        PIDController.setReference(
                                m_rollers.getEncoder().getPosition(),
                                ControlType.kPosition
                        );
                    }
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
        return run(() -> {
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

    private void toggleIntake() {
        solenoid.toggle();
        intakeRetracted = !intakeRetracted;
        // TODO: DO WE NEED THIS?
    }

    @Override
    public void periodic() {
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
        prevEncoder = m_rollers.getEncoder().getPosition();
    }
}
