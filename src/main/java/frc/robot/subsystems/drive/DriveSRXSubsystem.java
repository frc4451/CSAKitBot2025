package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSRXSubsystem extends SubsystemBase {
    // Instancing motor controllers
    private final TalonSRX leftLeader = new TalonSRX(DriveConstants.kFrontLeftId);
    private final TalonSRX rightLeader = new TalonSRX(DriveConstants.kFrontRightId);
    private final TalonSRX leftFollower = new TalonSRX(DriveConstants.kBackLeftId);
    private final TalonSRX rightFollower = new TalonSRX(DriveConstants.kBackRightId);



    public DriveSRXSubsystem() {
        configureMotor(leftLeader, false);
        configureMotor(leftFollower, false);
        leftFollower.follow(leftLeader);
        configureMotor(rightLeader, false);
        configureMotor(rightFollower, false);
        rightFollower.follow(rightLeader);
    }

    private void configureMotor(TalonSRX motor, boolean isInverted) {  
        motor.setInverted(isInverted);
        motor.setNeutralMode(NeutralMode.Brake);

        motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        DriveConstants.kCurrentLimit,
                        DriveConstants.kCurrentLimit,
                        0.04)); // 0.04 s is the same as the Phoenix 6 default of 40 ms

        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
    }

    @Override
    public void periodic() {
    }

    public Command driveOpenLoopCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(() -> {
            double speedVal = forward.getAsDouble();
            double rotationVal = rotation.getAsDouble();
            boolean allowTurnInPlace = speedVal == 0;
            WheelSpeeds drive = DifferentialDrive.arcadeDriveIK(speedVal, rotationVal, allowTurnInPlace);
            runOpenLoop(drive.left, drive.right);
        //     drive.curvatureDrive(speedVal, -rotationVal, allowTurnInPlace);
        }, this);
    }

    public void runOpenLoop(double leftPercent, double rightPercent) {
        leftLeader.set(TalonSRXControlMode.PercentOutput, leftPercent);
        rightLeader.set(TalonSRXControlMode.PercentOutput, rightPercent);
    }
}
