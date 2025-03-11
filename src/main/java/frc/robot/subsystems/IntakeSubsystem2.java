// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem2 extends SubsystemBase {

  public final TalonFX IntakeMotor1 = new TalonFX(Constants.IntakeConstants2.kIntakeMotorPort1);
  public final CANrange Rangefinder1 = new CANrange(Constants.IntakeConstants2.kCanRageID);
 // public final TalonFX IntakeMotor2 = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort2);
  /** Creates a new Intake. */
  public IntakeSubsystem2() {

    TalonFXConfiguration IntakeConfig1 = new TalonFXConfiguration();
    IntakeConfig1.MotorOutput.Inverted  = Constants.IntakeConstants2.kIntakeMotor1Inverted;
    IntakeConfig1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IntakeConfig1.HardwareLimitSwitch.ForwardLimitEnable =false;
    IntakeConfig1.HardwareLimitSwitch.ReverseLimitEnable = false;
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

  //ForwardLimitValue limitstate = IntakeMotor1.getForwardLimit().getValue();

  if (IntakeMotor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround){//Rangefinder1.getDistance().getValueAsDouble() < Constants.IntakeConstants.CanRangeDetectDistance) {
    return true;
  }
  else {
    return false;
  }
}


  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Object Detected", ObjectDetected());
    // This method will be called once per scheduler run
  }
}
