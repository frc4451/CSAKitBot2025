package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final int kFrontLeftId = 11;
    public static final int kFrontRightId = 12;
    public static final int kBackLeftId = 15;
    public static final int kBackRightId = 16;

    public static final double kRampRateSeconds = 0.6;
    public static final int kCurrentLimit = 60;

    
    public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);
    public static final double kMotorReduction = 8.45;

    public static final double kMaxSpeed = 5.2;

    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;

}
