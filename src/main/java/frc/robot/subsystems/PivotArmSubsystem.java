// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotArmSubsystem extends SubsystemBase {

  public final TalonFX PivotArmMotor1 = new TalonFX(Constants.PivotArmConstants.kPivotArmMotorPort);

private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public double lastsetpoint = 0;
  public double testSetpoint = 55;

  /** Creates a new Pivot_Arm. */
  public PivotArmSubsystem() {

    TalonFXConfiguration PivotArmConfig1 = new TalonFXConfiguration();
    PivotArmConfig1.MotorOutput.Inverted  = Constants.PivotArmConstants.kPivotArmMotorInverted;
    PivotArmMotor1.getConfigurator().apply(PivotArmConfig1);














  }

  public void SetPivotArmConfig1(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    // configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
    // configs.Slot1.kI = 0; // No output for integrated error
    // configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // // Peak output of 120 A
    // configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
    //   .withPeakReverseTorqueCurrent(Amps.of(-120));
      configs.MotorOutput.Inverted  = Constants.PivotArmConstants.kPivotArmMotorInverted;
      PivotArmMotor1.setNeutralMode(NeutralModeValue.Brake);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
   
    for (int i = 0; i < 5; ++i) {
      status = PivotArmMotor1.getConfigurator().apply(configs);
     
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString() );
    }
  }


  public double getVelocity(){

    return Math.abs(PivotArmMotor1.getVelocity().getValueAsDouble());
    
      }
    
      public void softlimitsOn(){
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Constants.PivotArmConstants.PivotArmMotorMaxSoftLimit);
        toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(Constants.PivotArmConstants.PivotArmMotorMinSoftLimit);
        toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
        PivotArmMotor1.getConfigurator().apply(toConfigure);
        
      }
    
      public void softlimitsOFF(){
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Constants.PivotArmConstants.PivotArmMotorMaxSoftLimit);
        toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(Constants.PivotArmConstants.PivotArmMotorMinSoftLimit);
        toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(false);
        toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(false);
        PivotArmMotor1.getConfigurator().apply(toConfigure);
        
      }
    
    public void BreakModeOn(boolean on){
    
    if (on){
      PivotArmMotor1.setNeutralMode(NeutralModeValue.Brake);
      
    }
    else{
      PivotArmMotor1.setNeutralMode(NeutralModeValue.Coast);
    
    }}
    
    
    
      public void Setspeed(double speed) {
        PivotArmMotor1.set(speed);
      
      }
    
      public void ZeroRotations() {
        PivotArmMotor1.setPosition(0);
      
      }
    
      public void updatelastsetpoint(double setpoint) {
        lastsetpoint = setpoint;
      }
    
      public void setPositionsetpoint(double setpoint) {
        PivotArmMotor1.setControl(m_positionVoltage.withPosition(setpoint));
        
      }
    
      public double getposition() {
      
        return PivotArmMotor1.getPosition().getValueAsDouble();
      }
    

  @Override
  public void periodic() {

      SmartDashboard.putNumber("Pivot Setpoint", lastsetpoint);
    SmartDashboard.putNumber("Pivot Motor Position", getposition());
    // This method will be called once per scheduler run
  }
}
