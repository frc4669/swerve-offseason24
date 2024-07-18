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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  // private final Drivetrain m_drivetrain = new Drivetrain(); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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

      m_swerveDrivetrain.drive(frc4669.squareInput(forward) * 0.9, frc4669.squareInput(strafe) * -0.9, rotation * -0.9);
    }, m_swerveDrivetrain));

    // m_driverController.a().onTrue(Commands.runOnce(() -> {
      // m_swerveDrivetrain.ZeroSwerveModules();
    // }, m_swerveDrivetrain));

    m_driverController.y().onTrue(Commands.runOnce(() -> m_swerveDrivetrain.resetSteeringPositions(), m_swerveDrivetrain));
    m_driverController.a().onTrue(Commands.runOnce(() -> m_swerveDrivetrain.resetAngle(), m_swerveDrivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
