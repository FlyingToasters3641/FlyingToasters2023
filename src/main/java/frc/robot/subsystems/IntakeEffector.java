package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import com.revrobotics.CANSparkMaxLowLevel;

public abstract class IntakeEffector extends SubsystemBase{

    private final DoubleSolenoid solenoid;
    private CANSparkMax m_rollers;
    private boolean intakeRetracted;

    public IntakeEffector() {
        solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            DrivetrainConstants.EFFECTOR_EXTENDED_CHANNEL, 
            DrivetrainConstants.EFFECTOR_RETRACTED_CHANNEL
        );

        solenoid.set(Value.kReverse);
        intakeRetracted = true;

        m_rollers = new CANSparkMax(DrivetrainConstants.ROLLER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public boolean isIn() {
        return intakeRetracted;
    }

    public Command runIntake() {
        return runOnce(() -> {
            m_rollers.set(0.3);
        });
        //TODO: MAP TO A BUTTON FOR RUNNING THE ROLLERS
    }

    public Command extendIntake() {
        return runOnce(() -> {
            if (solenoid.get() == Value.kReverse) {
                toggleIntake();
            }
        });
        //TODO: MAP TO BUTTON FOR EXTENDING THE WRIST
    }

    public Command stopIntake() {
        return runOnce(() -> {
            m_rollers.set(0);
        });
        //TODO: MAP TO BUTTON FOR STOPPING THE INTAKE
    }

    public Command retractIntake() {
        return runOnce(() -> {
            if (solenoid.get() == Value.kForward) {
                toggleIntake();
            }
        });
        //TODO: MAP TO BUTTON FOR RETRACTING THE WRIST
    }

    public Command reverseIntake() {
        return runOnce(() -> {
            m_rollers.set(-0.3);
        });
        //TODO: MAP TO BUTTON FOR REVERSING THE INTAKE
    }

    private void toggleIntake() {
        solenoid.toggle();
        intakeRetracted = !intakeRetracted;
        //DO WE NEED THIS?
    }
}