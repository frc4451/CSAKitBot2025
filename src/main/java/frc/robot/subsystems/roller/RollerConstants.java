package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RollerConstants {
    /*
     * Change this to what the real robot has
     */
    public static final int kCanID = 31; 
    public static final MotorType kMotorType = MotorType.kBrushed;

    public static final int kUpdatePeriodMilliseconds = 20;
    public static final double kMotorReduction = 1.0;
    public static final double kMoi = 1.0;
}
