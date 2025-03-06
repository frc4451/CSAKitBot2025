package frc.robot.subsystems.roller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkUtil;

public class RollerSubsystem extends SubsystemBase {
    private final SparkMax rollerMotor = new SparkMax(RollerConstants.kCanID, RollerConstants.kMotorType);
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    public RollerSubsystem() {
        configureMotorSettings();

    }

    /**
     * Configures motor settings for coral motor, can be tweaked for some tuning if
     * needed (Needed)
     */
    private void configureMotorSettings() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        config.encoder
                .uvwMeasurementPeriod(RollerConstants.kUpdatePeriodMilliseconds)
                .uvwAverageDepth(2);
        config.signals
                .primaryEncoderPositionPeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .primaryEncoderVelocityPeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .appliedOutputPeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .busVoltagePeriodMs(RollerConstants.kUpdatePeriodMilliseconds)
                .outputCurrentPeriodMs(RollerConstants.kUpdatePeriodMilliseconds);
        SparkUtil.tryUntilOk(
                rollerMotor,
                5,
                () -> rollerMotor.configure(
                        config,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

    }

    @Override
    public void periodic() {
        {
            String logRoot = getName();
            SmartDashboard.putNumber(logRoot + "Velocity", this.rollerEncoder.getVelocity());
            SmartDashboard.putNumber(logRoot + "AppliedVolts",
                    this.rollerMotor.getBusVoltage() * this.rollerMotor.getAppliedOutput());
            SmartDashboard.putNumber(logRoot + "CurrentAmps", this.rollerMotor.getOutputCurrent());
        }
    }

    public void runVolts(double volts) {
        rollerMotor.setVoltage(volts);
    }

    /**
     * Runs a specified voltage at `volts` until the Command ends
     */
    public Command runVoltsCommand(double volts) {
        return Commands.runEnd(() -> this.runVolts(volts), () -> this.runVolts(0));
    }
}
