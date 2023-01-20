// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.commands.Aadith_Autonomous;
import frc.robot.autonomous.commands.AntiSabatogeClimberFourBall;
import frc.robot.autonomous.commands.AutoRotateTest;
import frc.robot.autonomous.commands.testAuton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  //private final XboxController m_controller = new XboxController(0);
  private final PS4Controller m_controller = new PS4Controller(0);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);

  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);

  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_xspeedLimiter.calculate(-modifyAxis(m_controller.getLeftY())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_yspeedLimiter.calculate(-modifyAxis(m_controller.getLeftX())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_rotLimiter.calculate(-modifyAxis(m_controller.getRightX())) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    configAutonmousChooser();

    // Configure the button bindings
    configureButtonBindings();
  }

  public Command getAutonomousCommand() {
      return m_chooser.getSelected();
      
    }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller, Button.kA.value)
      .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.m_frontLeftModule.set(0, 90)));
    
  }

  private void configAutonmousChooser() { 
      SmartDashboard.putData("Chooser", m_chooser);
      m_chooser.setDefaultOption("rotate wheel", new InstantCommand(() -> m_drivetrainSubsystem.m_frontLeftModule.set(0, 90)));
      m_chooser.addOption("HAHA", new InstantCommand(() -> m_drivetrainSubsystem.m_frontLeftModule.set(0, Math.PI / 2)));
      m_chooser.addOption("Rotate Test", new AutoRotateTest(m_drivetrainSubsystem));
      m_chooser.addOption("Four Ball", new AntiSabatogeClimberFourBall(m_drivetrainSubsystem));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
