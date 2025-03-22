// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import static frc.robot.subsystems.arm.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
  private TalonSRX leftArmMotor;
  private TalonSRX rightArmMotor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    leftArmMotor = new TalonSRX(LEFT_ARM_MOTOR);
    rightArmMotor = new TalonSRX(RIGHT_ARM_MOTOR);

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);

    rightArmMotor.follow(leftArmMotor, FollowerType.PercentOutput);
    rightArmMotor.setInverted(InvertType.OpposeMaster);
  }

  //Moves the arm out at 25% of max output
  public void moveUp() {
    leftArmMotor.set(ControlMode.PercentOutput, 0.25);
  }

  //Moves the arm in at 25% of max output
  public void moveDown() {
    leftArmMotor.set(ControlMode.PercentOutput, -0.25);
  }

  //Stops the motors
  public void stop() {
    leftArmMotor.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
