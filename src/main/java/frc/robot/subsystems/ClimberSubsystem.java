// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  public final TalonFX ClimberMotor1 = new TalonFX(Constants.ClimberConstants.kClimberMotorPort);
    public TalonFXConfiguration Climberconfig1 = new TalonFXConfiguration();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    Climberconfig1.MotorOutput.Inverted  = Constants.ClimberConstants.kClimberMotorInverted;
    Climberconfig1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
ClimberMotor1.getConfigurator().apply(Climberconfig1);


  }


  public void Setspeed(double speed) {

      
       
    ClimberMotor1.set(speed);
  

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
