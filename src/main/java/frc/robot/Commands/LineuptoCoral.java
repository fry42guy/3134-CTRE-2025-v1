// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineuptoCoral extends Command {

private final CommandSwerveDrivetrain m_SwerveDrivetrain;
private boolean leftside;
private LimelightHelpers m_LimelightHelpers = new LimelightHelpers();
private Pose2d m_Pose2d = new Pose2d();
private SwerveRequest.RobotCentric m_SwerveRequest = new SwerveRequest.RobotCentric();

private PIDController m_xController = new PIDController(0.1, 0, 0);
private PIDController m_yController = new PIDController(0.1, 0, 0);
private PIDController m_thetaController = new PIDController(0.1, 0, 0);



  /** Creates a new LineuptoCoral. */
  public LineuptoCoral(CommandSwerveDrivetrain m_SwerveDrivetrain, boolean leftside) {
    this.m_SwerveDrivetrain = m_SwerveDrivetrain;
    this.leftside = leftside;
    addRequirements(m_SwerveDrivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Pose2d = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toPose2d();



    m_SwerveDrivetrain.setControl(m_SwerveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0));

   
      // Drivetrain will execute this command periodically
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
