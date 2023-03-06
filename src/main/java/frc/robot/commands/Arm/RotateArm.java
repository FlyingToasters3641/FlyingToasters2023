package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateArm extends CommandBase {
    private final Arm m_arm;

    private double m_position;

    public RotateArm(Arm arm, double position) {
        m_arm = arm;
        m_position = position;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
     
    }

    @Override
    public boolean isFinished() {
    return true;
    }

    @Override
    public void end(boolean interrupted) {
    
    }
}
