// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;




public class ElevatorSubsytem extends SubsystemBase {

  public final TalonFX elevatorMotor1 = new TalonFX(Constants.ElevatorConstants.kElevatorMotorPort);
  public final TalonFX elevatorMotor2 = new TalonFX(Constants.ElevatorConstants.kElevatorMotorPort2);
 
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public double lastsetpoint = 0;
  public double testSetpoint = 55;

 
  
   
  
  /** Creates a new Elevator. 
   * 
   * Discription: This class is used to control the elevator of the robot. It will have 2 motors, one on each side of the elevator. These motors will be Falcon 500s.
  */
  public ElevatorSubsytem() {

    
    
    
  

        TalonFXConfiguration elevatorConfig1 = new TalonFXConfiguration();
    elevatorConfig1.MotorOutput.Inverted  = Constants.ElevatorConstants.kElevatorMotorInverted;
    elevatorMotor1.getConfigurator().apply(elevatorConfig1);

    TalonFXConfiguration elevatorConfig2 = new TalonFXConfiguration();
    elevatorConfig2.MotorOutput.Inverted = Constants.ElevatorConstants.kElevatorMotor2Inverted;
    elevatorMotor2.getConfigurator().apply(elevatorConfig2);



    SetElevatorConfig1();
    SetElevatorConfig2();
  }

  public void SetElevatorConfig1(){
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
      configs.MotorOutput.Inverted  = Constants.ElevatorConstants.kElevatorMotorInverted;
      elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
   
    for (int i = 0; i < 5; ++i) {
      status = elevatorMotor1.getConfigurator().apply(configs);
     
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString() );
    }
  }

  public void SetElevatorConfig2(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = Constants.ElevatorConstants.Elevatorkp; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = Constants.ElevatorConstants.Elevatorki; // No output for integrated error
    configs.Slot0.kD = Constants.ElevatorConstants.Elevatorkd; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(Constants.ElevatorConstants.PeakForwardVoltage))
      .withPeakReverseVoltage(Volts.of(Constants.ElevatorConstants.PeakReverseVoltage));

    // configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
    // configs.Slot1.kI = 0; // No output for integrated error
    // configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // // Peak output of 120 A
    // configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
    //   .withPeakReverseTorqueCurrent(Amps.of(-120));
      configs.MotorOutput.Inverted  = Constants.ElevatorConstants.kElevatorMotorInverted;

      elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
   
    for (int i = 0; i < 5; ++i) {
      status = elevatorMotor2.getConfigurator().apply(configs);
     
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 2: " + status.toString() );
    }
  }

  public double getVelocity(){

return Math.abs(elevatorMotor1.getVelocity().getValueAsDouble());

  }

  public void softlimitsOn(){
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Constants.ElevatorConstants.ElevatorMotorMaxSoftLimit);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(Constants.ElevatorConstants.ElevatorMotorMinSoftLimit);
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    elevatorMotor1.getConfigurator().apply(toConfigure);
    elevatorMotor2.getConfigurator().apply(toConfigure);
  }

  public void softlimitsOFF(){
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Constants.ElevatorConstants.ElevatorMotorMaxSoftLimit);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(Constants.ElevatorConstants.ElevatorMotorMinSoftLimit);
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(false);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(false);
    elevatorMotor1.getConfigurator().apply(toConfigure);
    elevatorMotor2.getConfigurator().apply(toConfigure);
  }

public void BreakModeOn(boolean on){

if (on){
  elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
  elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
}
else{
  elevatorMotor2.setNeutralMode(NeutralModeValue.Coast);
  elevatorMotor2.setNeutralMode(NeutralModeValue.Coast);
}}



  public void Setspeed(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(speed);
  }

  public void ZeroRotations() {
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);
  }

  public void updatelastsetpoint(double setpoint) {
    lastsetpoint = setpoint;
  }

  public void setPositionsetpoint(double setpoint) {
    elevatorMotor1.setControl(m_positionVoltage.withPosition(setpoint));
    elevatorMotor2.setControl(m_positionVoltage.withPosition(setpoint));
  }

  public double getposition() {
  
    return elevatorMotor1.getPosition().getValueAsDouble();
  }

  // public double gettestSetpoint() {

  //   Double myvalue  = SmartDashboard.getNumber("TestSetpoint",100);

  //   System.out.println(myvalue);


  //   return myvalue;
  // }

  @Override
  public void periodic() {


    //SmartDashboard.putNumber("Current Setpoint", lastsetpoint);
    //SmartDashboard.putNumber("Current Motor Position", getposition());

    // This method will be called once per scheduler run
  }




}
