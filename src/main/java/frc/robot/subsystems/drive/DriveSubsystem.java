package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Instancing motor controllers
    private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.kFrontLeftId, MotorType.kBrushed);
    private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.kFrontRightId, MotorType.kBrushed);
    private final CANSparkMax backLeft = new CANSparkMax(DriveConstants.kBackLeftId, MotorType.kBrushed);
    private final CANSparkMax backRight = new CANSparkMax(DriveConstants.kBackRightId, MotorType.kBrushed);

    private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

    public DriveSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        // Reset the motors configuration; preparing for further configuration below
        frontLeft.restoreFactoryDefaults();
        frontRight.restoreFactoryDefaults();
        backLeft.restoreFactoryDefaults();
        backRight.restoreFactoryDefaults();

        // Make back motors run at the same velocity as the front motors
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        // Apply ramp rate
        frontLeft.setClosedLoopRampRate(DriveConstants.kRampRateSeconds);
        frontRight.setClosedLoopRampRate(DriveConstants.kRampRateSeconds);
        backLeft.setClosedLoopRampRate(DriveConstants.kRampRateSeconds);
        backRight.setClosedLoopRampRate(DriveConstants.kRampRateSeconds);

        // Write the configuration to the motor
        // Everything before this is preparing, only with this call will the motors
        // actually follow the configuration
        frontLeft.burnFlash();
        frontRight.burnFlash();
        backLeft.burnFlash();
        backRight.burnFlash();
    }

    /**
     * Method to run drive train in velocity control mode using "curve" arcade
     * Curve arcade sets the turn proportional to the forward/back joystick
     * ... therefore when forward is zero, turn is zero
     * This is the default drive method in tele-op periodic
     * 
     * @param speed            The robot's speed along the X axis [-1.0..1.0].
     *                         Forward is positive.
     * @param rotation         The normalized curvature [-1.0..1.0].
     *                         Counterclockwise is positive.
     * @param allowTurnInPlace If set, overrides constant-curvature turning for
     *                         turn-in-place maneuvers.
     *                         rotation will control turning rate instead of
     *                         curvature.
     */
    public Command driveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
        return Commands.run(() -> drive.curvatureDrive(speed.getAsDouble(), rotation.getAsDouble(), false), this);
    }

    public Command driveArcadeCommand(DoubleSupplier speed, DoubleSupplier rotation) {
        return Commands.run(() -> drive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble(), false), this);
    }

}
