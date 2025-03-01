package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkUtil;

public class DriveSubsystem extends SubsystemBase {
    // Instancing motor controllers
    private final SparkMax leftLeader = new SparkMax(DriveConstants.kFrontLeftId, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(DriveConstants.kFrontRightId, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(DriveConstants.kBackLeftId, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(DriveConstants.kBackRightId, MotorType.kBrushless);

    private final SparkClosedLoopController leftController = leftLeader.getClosedLoopController();
    private final SparkClosedLoopController rightController = rightLeader.getClosedLoopController();

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

    public DriveSubsystem() {
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

        SparkUtil.tryUntilOk(
        leftLeader,
        5,
        () -> leftEncoder.setPosition(0));

        SparkUtil.tryUntilOk(
        rightLeader,
        5,
        () -> rightEncoder.setPosition(0));
    }

    @Override
    public void periodic() {
        {
            String logRoot = getName() + "/Left/";
            SmartDashboard.putNumber(logRoot + "Velocity", this.leftEncoder.getVelocity());
            SmartDashboard.putNumber(logRoot + "AppliedVolts", this.leftLeader.getBusVoltage() * this.leftLeader.getAppliedOutput());
            SmartDashboard.putNumberArray(logRoot + "CurrentAmps", new double[] { 
                leftLeader.getOutputCurrent(), 
                leftFollower.getOutputCurrent()
            });
        }
        {
            String logRoot = getName() + "/Right/";
            SmartDashboard.putNumber(logRoot + "Velocity", this.rightEncoder.getVelocity());
            SmartDashboard.putNumber(logRoot + "AppliedVolts", this.rightLeader.getBusVoltage() * this.rightLeader.getAppliedOutput());
            SmartDashboard.putNumberArray(logRoot + "CurrentAmps", new double[] { 
                rightLeader.getOutputCurrent(), 
                rightFollower.getOutputCurrent()
            });
        }

    }

    public void driveClosedLoop(double forward, double rotation) {
        WheelSpeeds speeds = (forward != 0)
            ? DifferentialDrive.curvatureDriveIK(
                forward * Math.abs(forward),
                rotation * Math.abs(rotation) / 1.4, false)
            : DifferentialDrive.arcadeDriveIK(
                0,
                rotation / 2,
                true);
        
        runClosedLoop(
                speeds.left * DriveConstants.kMaxSpeed,
                speeds.right * DriveConstants.kMaxSpeed);
    }
    
    /** Command for controlling to drivetrain */
    public Command driveClosedLoopCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(() -> driveClosedLoop(forward.getAsDouble(), rotation.getAsDouble()), this);
    }

    public void runOpenLoop(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
        double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
        double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
        
        this.setVelocity(leftRadPerSec, rightRadPerSec);
    }

    public void setVelocity(double leftRadPerSec, double rightRadPerSec) {
        leftController.setReference(leftRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rightController.setReference(rightRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
}
