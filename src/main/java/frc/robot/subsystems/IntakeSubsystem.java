// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX IntakeMotor1 = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort1);
  public final CANrange Rangefinder1 = new CANrange(Constants.IntakeConstants.kCanRageID);
 // public final TalonFX IntakeMotor2 = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort2);
  /** Creates a new Intake. */
  public IntakeSubsystem() {

    TalonFXConfiguration IntakeConfig1 = new TalonFXConfiguration();
    IntakeConfig1.MotorOutput.Inverted  = Constants.IntakeConstants.kIntakeMotor1Inverted;
    IntakeConfig1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IntakeMotor1.getConfigurator().apply(IntakeConfig1);

CANrangeConfiguration RangeConfig1 = new CANrangeConfiguration();
RangeConfig1.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; //default 2500

Rangefinder1.getConfigurator().apply(RangeConfig1);




   // TalonFXConfiguration IntakeConfig2 = new TalonFXConfiguration();
   // IntakeConfig2.MotorOutput.Inverted  = Constants.IntakeConstants.kIntakeMotor2Inverted;
   // IntakeMotor2.getConfigurator().apply(IntakeConfig2);
  }


public void Setspeed(double speed) {
  IntakeMotor1.set(speed);
  //IntakeMotor2.set(speed);
}

public boolean ObjectDetected() {
  if (Rangefinder1.getDistance().getValueAsDouble() < Constants.IntakeConstants.CanRangeDetectDistance) {
    return true;
  }
  else {
    return false;
  }}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
