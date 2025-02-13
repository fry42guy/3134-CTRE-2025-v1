// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX IntakeMotor1 = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort1);
 // public final TalonFX IntakeMotor2 = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort2);
  /** Creates a new Intake. */
  public IntakeSubsystem() {

    TalonFXConfiguration IntakeConfig1 = new TalonFXConfiguration();
    IntakeConfig1.MotorOutput.Inverted  = Constants.IntakeConstants.kIntakeMotor1Inverted;
    IntakeMotor1.getConfigurator().apply(IntakeConfig1);
   // TalonFXConfiguration IntakeConfig2 = new TalonFXConfiguration();
   // IntakeConfig2.MotorOutput.Inverted  = Constants.IntakeConstants.kIntakeMotor2Inverted;
   // IntakeMotor2.getConfigurator().apply(IntakeConfig2);
  }


public void Setspeed(double speed) {
  IntakeMotor1.set(speed);
  //IntakeMotor2.set(speed);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
