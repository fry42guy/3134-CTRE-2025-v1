// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.IntSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Apriltagtracker extends SubsystemBase {

public double d_tagID;
public Integer lastTagID = -2;




  /** Creates a new Apriltagtracker. */
  public Apriltagtracker() {

SmartDashboard.putNumber("lastTagID", -2);




  }




 public int convertIDtoInt(double tagdouble ){
   Integer tagint = (int) tagdouble;

    return tagint;

 }

 public IntSupplier getTagID(){

  IntSupplier tagID = () -> lastTagID;

   return tagID;
 }


  @Override
  public void periodic() {


//System.out.println(Utils.isSimulation());

if (Utils.isSimulation()){
 // System.out.println("lastTagID: " + lastTagID);
  lastTagID = convertIDtoInt(SmartDashboard.getNumber("lastTagID", -1.0));

}
else{
  lastTagID = convertIDtoInt(LimelightHelpers.getFiducialID("limelight"));

}



    // This method will be called once per scheduler run
  }
}
