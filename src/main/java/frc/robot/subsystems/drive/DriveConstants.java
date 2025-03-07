package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    /*
     * Change these to what the real robot has
     */
    public static final int kFrontLeftId = 4;
    public static final int kFrontRightId = 1;
    public static final int kBackLeftId = 3;
    public static final int kBackRightId = 2;
    public static final MotorType kMotorType = MotorType.kBrushed;

    public static final double kRampRateSeconds = 0.6;
    public static final int kCurrentLimit = 60;
    
    public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);
    public static final double kMotorReduction = 8.45;

    public static final double kMaxSpeed = 5.2;

    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;
}
