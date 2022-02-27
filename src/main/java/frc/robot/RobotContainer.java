// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.LeaveTarmak;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ClimberSubsystem m_climber = new ClimberSubsystem();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // A complex auto routine that drives forward, drops a hatch, and then drives
        // backward.
        private final Command m_simpleAuto = new LeaveTarmak(m_robotDrive);

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                m_robotDrive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                new RunCommand(
                                                () -> m_robotDrive.arcadeDrive(
                                                                m_driverController.getLeftY(),
                                                                m_driverController.getRightX()),
                                                m_robotDrive));

                m_climber.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                new RunCommand(() -> m_climber.climb(m_driverController.getRawAxis(2)),
                                                m_climber));

                // Add commands to the autonomous command chooser
                m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
                m_chooser.addOption("Do Nothing", new InstantCommand());

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(m_chooser);
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Drive at half speed when the right bumper is held
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .whenPressed(() -> m_robotDrive.setMaxOutput(Constants.DriveConstants.kDriveTurboSpeed))
                                .whenReleased(() -> m_robotDrive.setMaxOutput(Constants.DriveConstants.kDriveMaxSpeed));

                new JoystickButton(m_driverController, Button.kY.value)
                                .whenPressed(() -> m_climber.resetEncoder());

                new JoystickButton(m_driverController, Button.kX.value)
                                .whenPressed(() -> m_robotDrive.setBrakeMode(true))
                                .whenReleased(() -> m_robotDrive.setBrakeMode(false));

                new JoystickButton(m_driverController, Button.kB.value)
                                .whenPressed(new LeaveTarmak(m_robotDrive));

                /*
                new POVButton(m_driverController, 0).whenPressed(() -> m_climber.climb(0.5))
                                .whenReleased(() -> m_climber.climb(0));

                new POVButton(m_driverController, 180).whenPressed(() -> m_climber.climb(-0.3))
                                .whenReleased(() -> m_climber.climb(0));
                */
        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new DriveToDistance(96, m_robotDrive);
        }
}
