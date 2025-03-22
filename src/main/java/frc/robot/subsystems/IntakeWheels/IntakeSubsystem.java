// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeWheels;

import static frc.robot.subsystems.IntakeWheels.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private VictorSPX intakeWheels;

  public IntakeSubsystem() {
   
    intakeWheels = new VictorSPX(WHEELS_ID); 

  }

  public void spinIntake(){

    intakeWheels.set(ControlMode.PercentOutput, 0.5);

  }


  public void spinOuttake(){

    intakeWheels.set(ControlMode.PercentOutput, -0.5);

  }

  public void stop(){
    intakeWheels.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
