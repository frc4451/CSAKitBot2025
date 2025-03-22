// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.drive.DriveREVSubsystem;
import frc.robot.subsystems.drive.DriveSPXSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.IntakeWheels.IntakeSubsystem;

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

    private final XboxController driverController = new XboxController(ControllerConstants.kDriverControllerPort);

    // private final DriveREVSubsystem driveSubsystem = new DriveREVSubsystem();
    private final DriveSPXSubsystem driveSubsystem = new DriveSPXSubsystem();
    // private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final JoystickButton moveOutButton = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton moveInButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    //for the intake wheels
    private final JoystickButton spinIntakeButton = new JoystickButton(driverController, XboxController.Button.kA.value);
    private final JoystickButton spinOuttakeButton = new JoystickButton(driverController, XboxController.Button.kY.value);

    //Moves the arm outwards while left bumper is held; stops the motor once left bumper is released
    private final Command moveOut = Commands.startEnd(() -> {armSubsystem.moveUp();}, () -> {armSubsystem.stop();}, armSubsystem);

    //Moves the arm inwards while right bumper is held; stops the motor once right bumper is released
    private final Command moveIn = Commands.startEnd(() -> {armSubsystem.moveDown();}, () -> {armSubsystem.stop();}, armSubsystem);
    
    private final Command spinIntake = Commands.startEnd(()->{intakeSubsystem.spinIntake();}, () -> {intakeSubsystem.stop();}, intakeSubsystem);

    private final Command spinOuttake = Commands.startEnd(()->{intakeSubsystem.spinOuttake();}, () -> {intakeSubsystem.stop();}, intakeSubsystem);


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveOpenLoopCommand(
                        // Negate because on controllers up is negative; up should be positive
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getRightY()));

        // driverController.rightTrigger().whileTrue(rollerSubsystem.runVoltsCommand(3));
        // driverController.leftTrigger().whileTrue(rollerSubsystem.runVoltsCommand(-3));

        moveOutButton.whileTrue(moveOut);
        moveInButton.whileTrue(moveIn);

        spinIntakeButton.whileTrue(spinIntake);
        spinOuttakeButton.whileTrue(spinOuttake);
    }

    public Command getAutonomousCommand() {
        return driveSubsystem.driveOpenLoopCommand(() -> 0.5, () -> 0).withTimeout(1);
        // return Commands.print("No autonomous command configured");
    }
}
