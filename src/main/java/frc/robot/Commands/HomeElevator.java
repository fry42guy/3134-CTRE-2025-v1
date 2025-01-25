// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class HomeElevator extends Command {

  private final ElevatorSubsytem m_elevator;

  /** Creates a new HomeElevator. */
  public HomeElevator(ElevatorSubsytem m_elevator) {
this.m_elevator = m_elevator;

    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.softlimitsOFF();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_elevator.Setspeed(Constants.ElevatorConstants.homespeed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

m_elevator.ZeroRotations();
m_elevator.updatelastsetpoint(0);
    m_elevator.Setspeed(0.0);
    m_elevator.softlimitsOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

if (m_elevator.getVelocity() < 0.001) {
  return true;
}
else {
    return false;
  }
  }
}
