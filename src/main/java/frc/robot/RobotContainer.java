package frc.robot;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.TeleopDriveConstants;
import frc.robot.autonomous.commands.testAuton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public Alliance alliance = Alliance.Invalid;

    /* Controllers */
    // private final Joystick driver = new Joystick(0);
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    
    Trigger leftTriggerO = operatorController.leftTrigger(); // Creates a new JoystickButton object for the `left bumper` button on exampleController
    
    Trigger rightTriggerO = operatorController.rightTrigger(); 
    
    Trigger rightTriggerD = driveController.rightTrigger(); 
    
    Trigger rightBumperD = driveController.rightBumper(); 

    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver,
    // XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Timer reseedTimer = new Timer();

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(
            /* photonCamera, */ m_drivetrainSubsystem);
    
    private final Arm m_Arm = new Arm();
    private final IntakeEffector m_intake = new IntakeEffector();

    /* UI Elements */
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_drivetrainSubsystem.setDefaultCommand(
                new FieldOrientedDriveCommand(
                        m_drivetrainSubsystem,
                        () -> m_poseEstimator.getCurrentPose().getRotation(),
                        () -> -modifyAxis(driveController.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(driveController.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(driveController.getRightX())
                                * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2));

        // Configure the button bindings
        configureButtonBindings();
        // Configure the autonomous chooser
        configureAutonomousChooser();

        reseedTimer.start();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // driveController.y().onTrue(new SequentialCommandGroup(
        //         m_Arm.moveArm(ArmPos.STORED_POSITION)));
        // driveController.x().onTrue(new SequentialCommandGroup(
        //         m_Arm.moveArm(ArmPos.GROUND_INTAKE_POSITION)));
        
        //Actual TODO: ADD SLOW MODE AND BALENCE
        operatorController.y().onTrue(new SequentialCommandGroup(
                 m_Arm.moveArm(ArmPos.DOUBLE_PLAYERSTATION_PICKUP)));
        operatorController.a().onTrue(new SequentialCommandGroup(
                 m_Arm.moveArm(ArmPos.STORED_POSITION)));
        operatorController.b().onTrue(new SequentialCommandGroup(
                m_Arm.moveArm(ArmPos.GROUND_INTAKE_POSITION)));
        operatorController.x().onTrue(new SequentialCommandGroup(
                m_Arm.moveArm(ArmPos.SOLO_PLAYERSTATION_PICKUP)));

        leftTriggerO.whileTrue(new SequentialCommandGroup(
            m_Arm.moveArm(ArmPos.L2_SCORING)));

        rightTriggerO.whileTrue(new SequentialCommandGroup(
            m_Arm.moveArm(ArmPos.L3_SCORING)));

        rightTriggerD.whileTrue(m_intake.runIntake());//run intake forward

        rightBumperD.whileTrue(m_intake.reverseIntake());//run intake backward

        driveController.x().whileTrue(m_Arm.extend(true));

        

        //Driver Buttons 
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    private Map<String, Command> eventMap = Map.of(
        // "extendHigh",
        // new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.HIGH_CUBE)
        //     .alongWith(new RunIntakeCommand(intakeSubsystem))
        //     .withTimeout(1.5),
        // "extendMid",
        // new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.MID_CUBE)
        //     .withTimeout(1),
        // "outtake",
        // new WaitCommand(0.5).deadlineWith(new ReverseIntakeCommand(intakeSubsystem)),
        // "extendIn",
        // new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, new ArmSetpoint(30, 0, 45))
        //     .withTimeout(0.5).andThen(
        //         new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.TRANSIT)
        //             .withTimeout(0.5)),
        // // "autoBalance",
        // // new AutoBalance(drivetrainSubsystem, poseEstimator));
        // "autoBalance",
        // new AutoBalance(drivetrainSubsystem, poseEstimator),
        // "goToIntakeMode",
        // new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem,
        //     Constants.ArmSetpoints.GROUND_CUBE_PICKUP),
        // "startIntake",
        // new WaitCommand(1.5).deadlineWith(new RunIntakeCommand(intakeSubsystem)),
        // "cone",
        // new InstantCommand(() -> {
        //   PiecePicker.toggle(false);
        //   ledSubsystem.setGamePiece(GamePiece.CONE);
        // }),
        // "cube",
        // new InstantCommand(() -> {
        //   PiecePicker.toggle(true);
        //   ledSubsystem.setGamePiece(GamePiece.CUBE);
        // })
        );
  
    private void configureAutonomousChooser() {
        SmartDashboard.putData("Chooser", chooser);
        // chooser.setDefaultOption("TestAuton", new testAuton(m_drivetrainSubsystem, m_poseEstimator));

        chooser.setDefaultOption("TestAuton",
            makeAutoBuilderCommand("New Path", new PathConstraints(1.5, 1)));
 
         //chooser.setDefaultOption("rotate wheel", new RunCommand(() -> {
        // SwerveModuleState[] moduleStates = s_Swerve.getModuleStates();
        // System.out.println("CURRENT: " + moduleStates.toString());

        // s_Swerve.setModuleStates(new SwerveModuleState[]{
        // new SwerveModuleState(0.5, Rotation2d.fromDegrees(45)),
        // new SwerveModuleState(0.5, Rotation2d.fromDegrees(315)),
        // new SwerveModuleState(0.5, Rotation2d.fromDegrees(135)),
        // new SwerveModuleState(0.5, Rotation2d.fromDegrees(225))
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
        // return new testAuton(s_Swerve);
    }

    public void disabledPeriodic() {
        // Reseed the motor offset continuously when the robot is disabled to help solve
        // dead wheel issue
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

    private CommandBase makeAutoBuilderCommand(String pathName, PathConstraints constraints) {
        // return new PPAutoBuilder(drivetrainSubsystem, poseEstimator, pathName,
        //     constraints,
        //     true, eventMap);
        var path = PathPlanner.loadPath(pathName, constraints);
        
        m_poseEstimator.addTrajectory(path);
        // controllerCommand = DrivetrainSubsystem.followTrajectory(driveSystem,
        // poseEstimatorSystem, alliancePath);
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_poseEstimator::getCurrentPose,
            m_poseEstimator::setCurrentPose,
            Constants.DrivetrainConstants.KINEMATICS,
            Constants.PPAutoConstants.translationConstants,
            Constants.PPAutoConstants.rotationConstants,
            m_drivetrainSubsystem::setModuleStates,
            eventMap,
            true,
            m_drivetrainSubsystem);
        return autoBuilder.fullAuto(path);
    
      }
      
    public void onAllianceChanged(Alliance currentAlliance) {
        alliance = currentAlliance;
        // m_poseEstimator.setAlliance(currentAlliance);
      }
}
