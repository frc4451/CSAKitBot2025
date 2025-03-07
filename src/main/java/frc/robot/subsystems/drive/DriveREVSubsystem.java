package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkUtil;

public class DriveREVSubsystem extends SubsystemBase {
    // Instancing motor controllers
    private final SparkMax leftLeader = new SparkMax(DriveConstants.kFrontLeftId, DriveConstants.kMotorType);
    private final SparkMax rightLeader = new SparkMax(DriveConstants.kFrontRightId, DriveConstants.kMotorType);
    private final SparkMax leftFollower = new SparkMax(DriveConstants.kBackLeftId, DriveConstants.kMotorType);
    private final SparkMax rightFollower = new SparkMax(DriveConstants.kBackRightId, DriveConstants.kMotorType);

    public DriveREVSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        SparkBaseConfig config = new SparkMaxConfig();
        IdleMode mode = IdleMode.kBrake;

        config.openLoopRampRate(DriveConstants.kRampRateSeconds)
                .closedLoopRampRate(DriveConstants.kRampRateSeconds)
                .idleMode(mode)
                .voltageCompensation(12.0)
                .smartCurrentLimit(DriveConstants.kCurrentLimit);

        config.encoder
                .positionConversionFactor((2 * Math.PI) / DriveConstants.kMotorReduction)
                .velocityConversionFactor(((2 * Math.PI) / 60.0) / DriveConstants.kMotorReduction)
                .uvwMeasurementPeriod(20)
                .uvwAverageDepth(2);

        config.inverted(DriveConstants.kLeftInverted);

        SparkUtil.tryUntilOk(
                leftLeader,
                5,
                () -> leftLeader.configure(config, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        config.inverted(DriveConstants.kRightInverted);
        SparkUtil.tryUntilOk(
                rightLeader,
                5,
                () -> rightLeader.configure(config, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        config.inverted(DriveConstants.kLeftInverted)
                .follow(leftLeader, false);
        SparkUtil.tryUntilOk(
                leftFollower,
                5,
                () -> leftFollower.configure(config, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        config.inverted(DriveConstants.kRightInverted)
                .follow(rightLeader, false);
        SparkUtil.tryUntilOk(
                rightFollower,
                5,
                () -> rightFollower.configure(config, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

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
            runClosedLoop(drive.left, drive.right);
        //     drive.curvatureDrive(speedVal, -rotationVal, allowTurnInPlace);
        }, this);
    }

    public void runClosedLoop(double left, double right) {
        leftLeader.set(left);
        rightLeader.set(right);
    }

    public void runOpenLoop(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }
}
