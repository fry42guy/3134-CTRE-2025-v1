// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.net.Proxy;
import java.time.format.TextStyle;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.coralv2;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetPose_Coral extends Command {

  private boolean left;
  private boolean hastarget = false;
  private double Targetx;
  private double Targety;
  private double Targetz;

  private double Xtolerance;
  private double Ytolerance;
  private double Ztolerance;

  private double Drivekp;
  private double Driveki;
  private double Drivekd;

  private double yDrivekp;
  private double yDriveki;
  private double yDrivekd;


  private double Rotatekp;
  private double Rotateki;
  private double Rotatekd;

  private double DriveMaxSpeed;
  private double RotateMaxSpeed;

  private PIDController xController;
  private PIDController yController;
  private PIDController zController;

  private double currentx;
  private double currenty;
  private double currentz;

private double xspeed;
private double yspeed;
private double zspeed;





  Pose3d campose;

  private CommandSwerveDrivetrain m_swerveDrivetrain;

private SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
.withDeadband(Constants.coralv2.drivedeadband).withRotationalDeadband(Constants.coralv2.rotatedeadband)
             // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

          
  

  






  /** Creates a new TargetPose_Coral. */
  public TargetPose_Coral(boolean left, CommandSwerveDrivetrain m_swerveDrivetrain) {

    this.left = left;
    this.m_swerveDrivetrain = m_swerveDrivetrain;



    addRequirements(m_swerveDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (left) {
Targetx = Constants.coralv2.leftxtarget;
Targety = Constants.coralv2.leftytarget;
Targetz = Constants.coralv2.leftztarget;


    }
    else {


      Targetx = Constants.coralv2.rightxtarget;
      Targety = Constants.coralv2.rightytarget;
      Targetz = Constants.coralv2.rightztarget;
    }

    Xtolerance = Constants.coralv2.Xtolerance;
    Ytolerance = Constants.coralv2.Ytolerance;
    Ztolerance = Constants.coralv2.Ztolerance;

    Drivekp = Constants.coralv2.Drivekp;
    Driveki = Constants.coralv2.Driveki;
    Drivekd = Constants.coralv2.Drivekd;

    yDrivekp = Constants.coralv2.yDrivekp;
    yDriveki = Constants.coralv2.yDriveki;
    yDrivekd = Constants.coralv2.yDrivekd;

    Rotatekp = Constants.coralv2.Rotatekp;
    Rotateki = Constants.coralv2.Rotateki;
    Rotatekd = Constants.coralv2.Rotatekd;

    DriveMaxSpeed = Constants.coralv2.DriveMaxSpeed;
    RotateMaxSpeed = Constants.coralv2.RotateMaxSpeed;

    xController = new PIDController(Drivekp, Driveki, Drivekd);
    yController = new PIDController(yDrivekp, yDriveki, yDrivekd);
    zController = new PIDController(Rotatekp, Rotateki, Rotatekd);

    initcontrolers();

    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    updatecamera();

    updatecontrollers();


    SmartDashboard.putNumber("xspeed", xspeed);
    SmartDashboard.putNumber("yspeed", yspeed);
    SmartDashboard.putNumber("zspeed", zspeed);


    



    m_swerveDrivetrain.setControl(
     drive

     
    .withVelocityX(-xspeed) // Drive forward with negative Y (forward)
       .withVelocityY(yspeed) // Drive left with negative X (left)
       .withRotationalRate(zspeed)
       );

  

// else {
//   m_swerveDrivetrain.applyRequest(() ->
//   drive.withVelocityX(0) // Drive forward with negative Y (forward)
//       .withVelocityY(0) // Drive left with negative X (left)
//       .withRotationalRate(0));
// }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


public void updatecontrollers() {

if (hastarget){

xspeed = xController.calculate(currentx);
yspeed = yController.calculate(currenty);
zspeed = zController.calculate(currentz);

if (xController.atSetpoint()){
  xspeed=0;
}

if (yController.atSetpoint()){
  yspeed=0;
}
if (zController.atSetpoint()){
  zspeed=0;
}


if (xspeed > DriveMaxSpeed) {
  xspeed = DriveMaxSpeed;
}
if (xspeed < -DriveMaxSpeed) {
  xspeed = -DriveMaxSpeed;}

if (yspeed > DriveMaxSpeed) {
  yspeed = DriveMaxSpeed;}

if (yspeed < -DriveMaxSpeed) {
  yspeed = -DriveMaxSpeed;}

if (zspeed > RotateMaxSpeed) {
  zspeed = RotateMaxSpeed;}

if (zspeed < -RotateMaxSpeed) {
  zspeed = -RotateMaxSpeed;}

}
else{
xspeed =0;
yspeed =0;
zspeed = 0;
}




}

public void initcontrolers(){


xController.setSetpoint(Targetx);
yController.setSetpoint(Targety);
zController.setSetpoint(Targetz);

xController.setTolerance(Xtolerance);
yController.setTolerance(Ytolerance);
zController.setTolerance(Ztolerance);












}



public void updatecamera() {


  

  if (left) {

   campose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
 
  }
  else {
    campose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight-left");
 
   
  }

 

 if (campose.getZ()==0.0) {

  hastarget = false;

 }
 else {hastarget = true;}


 currentx = campose.getZ();
 currenty = campose.getX();
 currentz = campose.getRotation().getY();

SmartDashboard.putNumber("tx cam", campose.getX());
SmartDashboard.putNumber("ty cam", campose.getY());
SmartDashboard.putNumber("tz cam", campose.getZ());
SmartDashboard.putNumber("rx cam", campose.getRotation().getX());
SmartDashboard.putNumber("ry cam", campose.getRotation().getY());
SmartDashboard.putNumber("rz cam", campose.getRotation().getZ());






  // currentx = campose.getX();
  // currenty = campose.getZ();
  // currentz = campose.getRotation().getAngle();


}}