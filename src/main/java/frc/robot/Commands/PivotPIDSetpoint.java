// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.PivotArmSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotPIDSetpoint extends Command {

  private double Setpoint;
  private final PivotArmSubsystem m_PivotArm;

  /** Creates a new PivotPIDSetpoint. */
  public PivotPIDSetpoint(PivotArmSubsystem m_PivotArm, double Setpoint ) {

    this.Setpoint = Setpoint;
    this.m_PivotArm = m_PivotArm;
    addRequirements(m_PivotArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_PivotArm.updatelastsetpoint(Setpoint);
    m_PivotArm.setPositionsetpoint(Setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

if (m_PivotArm.getposition() < Setpoint + 0.1 && m_PivotArm.getposition() > Setpoint - 0.1) {

  return true;
}
  else{



    return false;
  }
}
}
