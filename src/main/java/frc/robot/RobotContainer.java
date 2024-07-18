// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Vision m_vision = new Vision();

  // subsystem init
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain(m_vision);
  // private final Drivetrain m_drivetrain = new Drivetrain(); 

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private SendableChooser<Command> m_autoChooser; 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // m_drivetrain.setDefaultCommand(new RunCommand(()->{
    //   m_drivetrain.SetSwerveOutput(m_driverController.getLeftX(), m_driverController.getRightY());
      
    // }, m_drivetrain));
    

    m_swerveDrivetrain.setDefaultCommand(new RunCommand(() -> {
      double forward = frc4669.ApplyJoystickDeadZone(m_driverController.getLeftY(), 0.05);
      double strafe = frc4669.ApplyJoystickDeadZone(m_driverController.getLeftX(), 0.05);
      double rotation = frc4669.ApplyJoystickDeadZone(m_driverController.getRightX(), 0.05);

      m_swerveDrivetrain.drive(forward * 0.3, strafe * -0.3, rotation * 0.3);
    }, m_swerveDrivetrain));

    // m_driverController.a().onTrue(Commands.runOnce(() -> {
      // m_swerveDrivetrain.ZeroSwerveModules();
    // }, m_swerveDrivetrain));


    this.m_autoChooser = AutoBuilder.buildAutoChooser(); // load in all the paths
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected(); 
  }
}
