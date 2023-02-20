package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.TeleopDriveConstants;
import frc.robot.autonomous.AutonomousCommand;
import frc.robot.autonomous.commands.testAuton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // private final Joystick driver = new Joystick(0);
    private final CommandXboxController controller = new CommandXboxController(0);

    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Timer reseedTimer = new Timer();

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(/* photonCamera, */ m_drivetrainSubsystem);

    /* UI Elements */
    private final SendableChooser<Command> chooser = new SendableChooser<>();
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_drivetrainSubsystem.setDefaultCommand(
            new FieldOrientedDriveCommand(
                m_drivetrainSubsystem,
                () -> m_poseEstimator.getCurrentPose().getRotation(),
                () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2));

        // Configure the button bindings
        configureButtonBindings();
        // Configure the autonomous chooser
         configureAutonomousChooser();

         reseedTimer.start();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    private void configureAutonomousChooser() {
        SmartDashboard.putData("Chooser", chooser);
        //chooser.setDefaultOption("TestAuton", new testAuton(s_Swerve));
        // chooser.setDefaultOption("rotate wheel", new RunCommand(() -> {
        //   SwerveModuleState[] moduleStates = s_Swerve.getModuleStates();
        //   System.out.println("CURRENT: " + moduleStates.toString());

        //   s_Swerve.setModuleStates(new SwerveModuleState[]{
        //     new SwerveModuleState(0.5, Rotation2d.fromDegrees(45)),
        //     new SwerveModuleState(0.5, Rotation2d.fromDegrees(315)),
        //     new SwerveModuleState(0.5, Rotation2d.fromDegrees(135)),
        //     new SwerveModuleState(0.5, Rotation2d.fromDegrees(225))
        // });
        // }));;
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        
        return chooser.getSelected();
        //return new testAuton(s_Swerve);
    }

    public void disabledPeriodic() {
        // Reseed the motor offset continuously when the robot is disabled to help solve dead wheel issue
        if (reseedTimer.advanceIfElapsed(1.0)) {
          m_drivetrainSubsystem.reseedSteerMotorOffsets();
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, TeleopDriveConstants.DEADBAND);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }    
}
