package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSPXSubsystem extends SubsystemBase {
    // Instancing motor controllers
    private final VictorSPX leftLeader = new VictorSPX(DriveConstants.kFrontLeftId);
    private final VictorSPX rightLeader = new VictorSPX(DriveConstants.kFrontRightId);
    private final VictorSPX leftFollower = new VictorSPX(DriveConstants.kBackLeftId);
    private final VictorSPX rightFollower = new VictorSPX(DriveConstants.kBackRightId);

    public DriveSPXSubsystem() {
        configureMotor(leftLeader, DriveConstants.kLeftInverted);
        configureMotor(leftFollower, DriveConstants.kLeftInverted);
        leftFollower.follow(leftFollower);
        configureMotor(rightLeader, DriveConstants.kRightInverted);
        configureMotor(rightFollower, DriveConstants.kRightInverted);
        rightFollower.follow(rightLeader);
    }

    private void configureMotor(VictorSPX motor, boolean isInverted) {  
        motor.setInverted(isInverted);
        motor.setNeutralMode(NeutralMode.Brake);

        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
    }

    @Override
    public void periodic() {
    }

    public Command driveOpenLoopCommand(DoubleSupplier forwardLeft, DoubleSupplier forwardRight) {
        return Commands.run(() -> {
            double leftSpeedVal = forwardLeft.getAsDouble();
            double rightSpeedVal = forwardRight.getAsDouble();
            boolean decreaseSensitivity = Math.abs(leftSpeedVal) < .2 && Math.abs(rightSpeedVal) < .2;
            WheelSpeeds drive = DifferentialDrive.tankDriveIK(leftSpeedVal, rightSpeedVal, decreaseSensitivity);
            runOpenLoop(drive.left, drive.right);
        }, this);
    }

    public void runOpenLoop(double leftPercent, double rightPercent) {
        leftLeader.set(VictorSPXControlMode.PercentOutput, MathUtil.clamp(leftPercent*2, -.58, .58));
        rightLeader.set(VictorSPXControlMode.PercentOutput, MathUtil.clamp(rightPercent*2, -.58, .58));
    }
}
