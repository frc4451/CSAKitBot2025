// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkUtil;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climberMotor;
  SparkMaxConfig climberConfig;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);

    climberConfig = new SparkMaxConfig();

    climberConfig
      .voltageCompensation(CLIMBER_VOLT_COMPENSATION)
      .smartCurrentLimit(CLIMBER_CURRENT_LIMIT)
      .idleMode(IdleMode.kBrake);

    SparkUtil.tryUntilOk(
        climberMotor,
        5,
        /*
         * Attempting to use kResetSafeParameters and kPersistParameters may cause it to timeout so
         * go into REV Hardware Client and set the current limit to 60 and idle mode to brake manually
         */
        () -> climberMotor.configure(climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
      );
  }

  //Moves the climber at the speed inputted
  public void climb(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
