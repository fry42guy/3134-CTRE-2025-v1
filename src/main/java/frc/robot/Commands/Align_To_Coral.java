// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Apriltagtracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align_To_Coral extends Command {

private int tagID; 
private final CommandSwerveDrivetrain m_swerve;
private final Apriltagtracker m_tagtracker;
private boolean Leftside = false;
private AprilTagFieldLayout  fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

private Command runcommand; 





  /** Creates a new Align_To_Coral. */
  public Align_To_Coral(Apriltagtracker m_tagtracker, CommandSwerveDrivetrain m_swerve, boolean Leftside) {
    
    this.m_tagtracker = m_tagtracker;
    this.m_swerve = m_swerve;
    this.Leftside = Leftside;
    addRequirements(m_swerve);
    addRequirements(m_tagtracker);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


tagID = m_tagtracker.getTagID().getAsInt();
System.out.println("tagID: " + tagID);
PathConstraints constraints = new PathConstraints(
        3, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));



        runcommand =   m_swerve.Autodrive( getTargetPose(tagID, Leftside), constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));

//  runcommand = AutoBuilder.pathfindToPose(
//        // pose,
//        getTargetPose(tagID, Leftside),
//         constraints,
//         edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
//     );


     runcommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (runcommand != null) {
      runcommand.cancel();  // Ensures the auto drive command stops when this command is interrupted
  }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return runcommand == null || runcommand.isFinished();
  }





public Pose2d getTargetPose(int ID , Boolean Leftside) {
    // Retrieve the AprilTag's pose


   

//int tagID = ID.intValue();

    Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(ID);
    if (tagPoseOptional.isEmpty()) {
        // Handle the case where the tag ID is not found
        return null;
    }
    Pose2d tagPose = tagPoseOptional.get().toPose2d();

    // Tag's position and orientation
    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagTheta = tagPose.getRotation().getRadians();

    // Offsets in meters
    double leftOffset = Units.inchesToMeters(Constants.LimeLightConstants.Left_offset);
    double backwardOffset = Units.inchesToMeters(-Constants.LimeLightConstants.Front_to_back_offset_in);

    if (!Leftside) {
        leftOffset = -Constants.LimeLightConstants.Right_offset;   }

    // Calculate the new position
    double newX = tagX - (backwardOffset * Math.cos(tagTheta)) + (leftOffset * Math.sin(tagTheta));
    double newY = tagY - (backwardOffset * Math.sin(tagTheta)) - (leftOffset * Math.cos(tagTheta));

    // The robot should face the same direction as the tag
    Rotation2d newRotation = tagPose.getRotation();//.plus(new Rotation2d(Units.degreesToRadians(180)));

    // Return the new pose
    return new Pose2d(newX, newY, newRotation);
}

}
