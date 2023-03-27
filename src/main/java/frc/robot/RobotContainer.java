package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPos;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.TeleopDriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.kArm;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.HashMap;
import java.util.Map;

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
  private final CommandXboxController driveController = new CommandXboxController(
    0
  );
  private final CommandXboxController operatorController = new CommandXboxController(
    1
  );

  Trigger leftTriggerO = operatorController.leftTrigger();

  Trigger rightTriggerO = operatorController.rightTrigger();

  Trigger rightTriggerD = driveController.rightTrigger();

  Trigger leftTriggerD = driveController.leftTrigger();

  private double joystickSensitivity = 1;

  // bumpers
  Trigger leftBumperO = operatorController.leftBumper();

  Trigger rightBumperO = operatorController.rightBumper();

  Trigger rightBumperD = driveController.rightBumper();

  Trigger leftBumperD = driveController.leftBumper();

  /* Subsystems */
  private final Timer reseedTimer = new Timer();

  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(
    /* photonCamera, */m_drivetrainSubsystem
  );
  private final IntakeEffector m_intake = new IntakeEffector();
  private final Arm m_Arm = new Arm(m_intake, m_LEDSubsystem);

  /* UI Elements */
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private Map<String, Command> eventMap = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      eventMap.put("ScoreL3", new ParallelCommandGroup(m_Arm.moveArm(ArmPos.L3_SCORING, 1), new SequentialCommandGroup(new WaitCommand(0.45), m_Arm.extend(kArm.EXTENDED_POSITION), new WaitCommand(0.15), m_intake.reverseIntake().withTimeout(0.3))));
      eventMap.put("AutoBalance", new AutoBalanceAlt(m_drivetrainSubsystem, m_poseEstimator));
      eventMap.put("ScoreL2", m_Arm.moveArm(ArmPos.L2_SCORING, 1)); //.andThen(m_intake.reverseIntake()).withTimeout(0.5).andThen(m_Arm.moveArm(ArmPos.STORED_POSITION)))
      eventMap.put("Extend", m_Arm.extend(ArmPos.L3_SCORING.getExtended()));
      eventMap.put("Outtake", m_intake.reverseIntake().withTimeout(0.5));
      eventMap.put("Retract", m_Arm.extend(0));
      eventMap.put("RetractToRest", m_Arm.moveArm(ArmPos.STORED_POSITION).withTimeout(0.75));
      eventMap.put("GroundIntake", new SequentialCommandGroup(m_Arm.moveArm(ArmPos.GROUND_INTAKE_POSITION, 1).withTimeout(0.5), m_Arm.extendGroundIntake(), m_intake.extendIntake(), m_intake.runIntake(m_LEDSubsystem)));
      eventMap.put("StartIntake", m_intake.runIntake(m_LEDSubsystem).withTimeout(2)); // TODO: Tune timeout! this should stop before we try to score
      eventMap.put("MoveArmUp", m_Arm.moveArm(ArmPos.SOLO_PLAYERSTATION_PICKUP));
      eventMap.put("MoveToL3", m_Arm.moveArm(ArmPos.L3_SCORING, 1));
      eventMap.put("MoveToL2", m_Arm.moveArm(ArmPos.L2_SCORING, 1));
    m_drivetrainSubsystem.setDefaultCommand(
      new FieldOrientedDriveCommand(
        m_drivetrainSubsystem,
        () -> m_poseEstimator.getCurrentPose().getRotation(),
        () ->
          -modifyAxis(driveController.getLeftY() * joystickSensitivity) *
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () ->
          -modifyAxis(driveController.getLeftX() * joystickSensitivity) *
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () ->
          -modifyAxis(driveController.getRightX() * joystickSensitivity) *
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND /
          2
      )
    );

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
    // OPPERATOR BUTTON BINDINGS
    operatorController
      .x()
      .onTrue(
        new SequentialCommandGroup(
          m_Arm.moveArm(ArmPos.SOLO_PLAYERSTATION_PICKUP)
        )
      )
      .onFalse(
        new SequentialCommandGroup(m_Arm.moveArm(ArmPos.STORED_POSITION))
      );
    
    operatorController
      .y()
      .onTrue(
        new ParallelCommandGroup(
          m_Arm.extendDoublePlayerStatiion(),
          m_Arm.moveArm(ArmPos.DOUBLE_PLAYERSTATION_PICKUP)
        )
      )
      .onFalse(
        new SequentialCommandGroup(m_Arm.moveArm(ArmPos.STORED_POSITION))
      );
    
    operatorController
      .b()
      .onTrue(new SequentialCommandGroup(m_Arm.moveArm(ArmPos.L2_SCORING)));
    
    operatorController
      .a()
      .onTrue(
        new SequentialCommandGroup(m_Arm.moveArm(ArmPos.STORED_POSITION))
      );

    leftTriggerO.onTrue(m_intake.runIntake(m_LEDSubsystem));

    rightTriggerO.whileTrue(m_Arm.extendL3()).whileFalse(m_Arm.extend(0));

    //Purple LED's
    operatorController
      .rightBumper()
      .onTrue(
        new InstantCommand(() -> {
          m_LEDSubsystem.ledSwitch(3);
        })
      )
      .onFalse(new InstantCommand(() -> m_LEDSubsystem.ledSwitch(1)));
    //Yellow LED's
    operatorController
      .leftBumper()
      .onTrue(
        new InstantCommand(() -> {
          m_LEDSubsystem.ledSwitch(2);
        })
      )
      .onFalse(new InstantCommand(() -> m_LEDSubsystem.ledSwitch(1)));

    // DRIVER BUTTON BINDINGS
    rightTriggerD.whileTrue(m_intake.reverseIntake());
    leftTriggerD.onTrue(new ParallelCommandGroup(
      m_Arm.extendGroundIntake(),
      m_Arm.moveArm(ArmPos.GROUND_INTAKE_POSITION)));
    // driveController.y().whileTrue(new AutoBalanceAlt(m_drivetrainSubsystem,
    // m_poseEstimator));

    // Field-oriented 90 degree mode
//     driveController
//         .rightStick().whileTrue(
//                 new FieldHeadingDriveCommand(
//                         m_drivetrainSubsystem,
//                         () -> m_poseEstimator.getCurrentPose().getRotation(),
//                         () -> -modifyAxis(driveController.getLeftY() * joystickSensitivity) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
//                         () -> -modifyAxis(driveController.getLeftX() * joystickSensitivity) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
//                         () -> -modifyAxis(driveController.getRightY()),
//                         () -> -modifyAxis(driveController.getRightX())
//                 )                
//         );
        // Controls:
        //   Y => Straight ahead drive station facing out (+x)
        //   X => Face right from driver's perspective (+y)
        //   A => Face back at driver station (-x)
        //   B => Face left from driver's perspective (-y)
        driveController
        .x().or(driveController.a()).or(driveController.b()).or(driveController.y()).whileTrue(
                new FieldHeadingDriveCommand(
                        m_drivetrainSubsystem,
                        () -> m_poseEstimator.getCurrentPose().getRotation(),
                        () -> -modifyAxis(driveController.getLeftY() * joystickSensitivity) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(driveController.getLeftX() * joystickSensitivity) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> driveController.getHID().getYButton() ? 1 : (driveController.getHID().getAButton() ? -1 : 0),
                        () -> driveController.getHID().getXButton() ? 1 : (driveController.getHID().getBButton() ? -1 : 0)
                )                
        );

    // SLOW MODE
    driveController
      .leftBumper()
      .onTrue(
        new InstantCommand(() -> {
          joystickSensitivity = 0.5;
        })
      )
      .onFalse(
        new InstantCommand(() -> {
          joystickSensitivity = 1.0;
        })
      );
  }

  //        private Map<String, Command> eventMap = Map.of(
  //                        "ScoreL3",
  //                        m_Arm.extend(ArmPos.L3_SCORING.getExtended())
  //                                        .andThen(m_intake.reverseIntake().withTimeout(.75)),
  //                        "AutoBalance", new AutoBalanceAlt(m_drivetrainSubsystem, m_poseEstimator),
  //                        "ScoreL2", m_Arm.moveArm(ArmPos.L2_SCORING), // .andThen(m_intake.reverseIntake()).withTimeout(0.5).andThen(m_Arm.moveArm(ArmPos.STORED_POSITION))
  //                        "Extend", m_Arm.extend(ArmPos.L3_SCORING.getExtended()),
  //                        "Outtake", m_intake.reverseIntake().withTimeout(0.5),
  //                        "Retract", m_Arm.extend(0),
  //                        "RetractToRest", m_Arm.moveArm(ArmPos.STORED_POSITION).withTimeout(0.75),
  //                        "GroundIntake",
  //                        new SequentialCommandGroup(m_Arm.moveArm(ArmPos.GROUND_INTAKE_POSITION),
  //                                        m_intake.runIntake(m_LEDSubsystem).withTimeout(4)),
  //                        "StartIntake", m_intake.runIntake(m_LEDSubsystem).withTimeout(2) // TODO: Tune timeout! this
  //                                                                                         // should stop before we try to
  //                                                                                         // score );
  //        );

  private void configureAutonomousChooser() {
    SmartDashboard.putData("Chooser", chooser);
    // chooser.setDefaultOption("TestAuton", new testAuton(m_drivetrainSubsystem, m_poseEstimator));

    chooser.setDefaultOption(
      "OneConeBalance",
      makeAutoBuilderCommand("1ConeBalance", new PathConstraints(1.5, 1))
    );

    chooser.addOption(
      "2GPBarrier",
      makeAutoBuilderCommand("2GPBarrier", new PathConstraints(3, 2))
    );

    chooser.addOption(
      "2GPWall",
      makeAutoBuilderCommand("2GPWall", new PathConstraints(3, 2))
    );
    chooser.addOption(
      "3GPBarrier",
      makeAutoBuilderCommand("3GPBarrier", new PathConstraints(3, 2))
    );
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

  private CommandBase makeAutoBuilderCommand(
    String pathName,
    PathConstraints constraints
  ) {
    // return new PPAutoBuilder(drivetrainSubsystem, poseEstimator, pathName,
    // constraints,
    // true, eventMap);
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
      m_drivetrainSubsystem
    );
    return autoBuilder.fullAuto(path);
  }

  public void onAllianceChanged(Alliance currentAlliance) {
    alliance = currentAlliance;
    // m_poseEstimator.setAlliance(currentAlliance);
  }
}
