// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.frc4669.Logging;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pivit;
import frc.robot.subsystems.Intake;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision;
import frc.robot.subsystems.Elevator;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // robot wide states
  boolean m_isJoyConEnabled = true; 

  // subsystem init
  private final Vision m_vision = new Vision();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain(m_vision);
  private final Pivit m_pivit = new Pivit();
  private final Elevator m_elevator = new Elevator(); 
  private final Intake m_intake = new Intake(); 
  // private final Drivetrain m_drivetrain = new Drivetrain(); 

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private SendableChooser<Command> m_autoChooser; 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    frc4669.Logging.StartLogger();
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {   
    // NOTE!!!!: Y axes are negative ON THE CONTROLLER, the others are not
    // Rotation should be CCW positive
    m_swerveDrivetrain.setDefaultCommand(new RunCommand(() -> {
      if (!m_isJoyConEnabled) return; // do nothing if joystick controls disabled

      double forward = frc4669.squareInput(frc4669.ApplyJoystickDeadZone(m_driverController.getLeftY(), 0.1)) * Constants.Swerve.kSpeedLimit * OperatorConstants.kMovementMultiplier;
      double strafe = -frc4669.squareInput(frc4669.ApplyJoystickDeadZone(m_driverController.getLeftX(), 0.1)) * Constants.Swerve.kSpeedLimit * OperatorConstants.kMovementMultiplier;
      double rotation = -frc4669.squareInput(frc4669.ApplyJoystickDeadZone(m_driverController.getRightX(), 0.1)) * Constants.Swerve.kMaxAngularSpeed * OperatorConstants.kRotationMultiplier;

      m_swerveDrivetrain.drive(forward, strafe, rotation);
    }, m_swerveDrivetrain));

    // Trigger on SmartDash Board button
    SmartDashboard.putBoolean("Joystick Control Enabled", m_isJoyConEnabled);
    new Trigger(() -> {return SmartDashboard.getBoolean("Joystick Control Enabled", true);})
      .onFalse(Commands.runOnce(() -> {m_isJoyConEnabled = false;}))
      .onTrue(Commands.runOnce(() -> {m_isJoyConEnabled = true;}));

    m_driverController.x().onTrue(m_swerveDrivetrain.ZeroSwerveModules()); 
    m_driverController.y().onTrue(Commands.runOnce(() -> m_swerveDrivetrain.resetSteeringPositions(), m_swerveDrivetrain));
    m_driverController.a().onTrue(Commands.runOnce(() -> m_swerveDrivetrain.resetAngle(), m_swerveDrivetrain));

    this.m_autoChooser = AutoBuilder.buildAutoChooser(); // load in all the paths
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); 
  }
}
