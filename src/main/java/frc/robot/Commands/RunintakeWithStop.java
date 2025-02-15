// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunintakeWithStop extends Command {

private boolean originalstateofcanrange;
private boolean DriveFwd;
//private double timeout;
private final IntakeSubsystem m_intake;


  /** Creates a new RunintakeWithStop. */
  public RunintakeWithStop(IntakeSubsystem m_intake, boolean DriveFwd) {

    this.m_intake = m_intake;
    this.DriveFwd = DriveFwd;
    addRequirements(m_intake);



    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

originalstateofcanrange = m_intake.ObjectDetected();

if (DriveFwd) {
  m_intake.Setspeed(Constants.IntakeConstants.FWDspeed);
}
else {
  m_intake.Setspeed(Constants.IntakeConstants.REVspeed);
}


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.Setspeed(0.0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!originalstateofcanrange && originalstateofcanrange != m_intake.ObjectDetected()) {
      return true;
    }
    else {
    return false;
  }
}
}
