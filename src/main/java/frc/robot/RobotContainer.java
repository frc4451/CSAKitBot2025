// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveREVSubsystem;
import frc.robot.subsystems.drive.DriveSPXSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    // private final CommandCustomXboxController driverController = new CommandCustomXboxController(ControllerConstants.kDriverControllerPort);
    // private final CommandJoystick driverController = new CommandJoystick(ControllerConstants.kDriverControllerPort);

    private final CommandCustomXboxController driverController = new CommandCustomXboxController(ControllerConstants.kDriverControllerPort);
    private final XboxController climbController = new XboxController(ControllerConstants.CLIMB_CONTROLLER_PORT);

    private final DriveREVSubsystem driveSubsystem = new DriveREVSubsystem();
    // private final DriveSPXSubsystem driveSubsystem = new DriveSPXSubsystem();
    // private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final Command moveClimberUp = Commands.startEnd(() -> {climberSubsystem.climb(0.5);}, () -> {climberSubsystem.climb(0);}, climberSubsystem);
    private final Command moveClimberDown = Commands.startEnd(() -> {climberSubsystem.climb(-0.5);}, () -> {climberSubsystem.climb(0);}, climberSubsystem);

    private final POVButton climberUpButton = new POVButton(climbController, 0);
    private final POVButton climberDownButton = new POVButton(climbController, 180);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveOpenLoopCommand(
                        // Negate because on controllers up is negative; up should be positive
                        () -> -driverController.getLeftY(),
                        () -> driverController.getRightX()));

        climberUpButton.whileTrue(moveClimberUp);
        climberDownButton.whileTrue(moveClimberDown);

        // driverController.rightTrigger().whileTrue(rollerSubsystem.runVoltsCommand(3));
        // driverController.leftTrigger().whileTrue(rollerSubsystem.runVoltsCommand(-3));
    }

    public Command getAutonomousCommand() {
        return driveSubsystem.driveOpenLoopCommand(() -> 0.5, () -> 0).withTimeout(1);
        // return Commands.print("No autonomous command configured");
    }
}
