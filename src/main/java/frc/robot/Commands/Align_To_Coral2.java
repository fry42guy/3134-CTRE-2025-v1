package frc.robot.Commands;
import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Apriltagtracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Align_To_Coral2 extends ProxyCommand {

private int tagID; 
private final CommandSwerveDrivetrain m_swerve;
private final Apriltagtracker m_tagtracker;
private boolean Leftside = false;
private AprilTagFieldLayout  fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);




    public Align_To_Coral2(Apriltagtracker m_tagtracker, CommandSwerveDrivetrain m_swerve, boolean Leftside) {
        super(() -> {
            int tagID = m_tagtracker.getTagID().getAsInt();
            System.out.println("tagID: " + tagID);

            PathConstraints constraints = new PathConstraints(
                3, 2,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

            return m_swerve.Autodrive(getTargetPose(tagID, Leftside), constraints, 
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
        });
        this.m_tagtracker = m_tagtracker;
        this.m_swerve = m_swerve;
        this.Leftside = Leftside;
    }

    private static Pose2d getTargetPose(int ID, boolean Leftside) {


        SmartDashboard.putNumber("Tag tracker Last", ID);


        Optional<Pose3d> tagPoseOptional = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(ID);
        if (tagPoseOptional.isEmpty()) {
            return new Pose2d();  // Return default pose if tag not found
        }

        Pose2d tagPose = tagPoseOptional.get().toPose2d();
        double tagX = tagPose.getX();
        double tagY = tagPose.getY();
        double tagTheta = tagPose.getRotation().getRadians();

        double leftOffset = Units.inchesToMeters(Constants.LimeLightConstants.Left_offset);
        double backwardOffset = Units.inchesToMeters(-Constants.LimeLightConstants.Front_to_back_offset_in);

        if (!Leftside) {
            leftOffset = Units.inchesToMeters(Constants.LimeLightConstants.Right_offset*-1);
        }

        System.out.println(leftOffset);

        double newX = tagX - (backwardOffset * Math.cos(tagTheta)) + (leftOffset * Math.sin(tagTheta));
        double newY = tagY - (backwardOffset * Math.sin(tagTheta)) - (leftOffset * Math.cos(tagTheta));
        Rotation2d newRotation = tagPose.getRotation();

        return new Pose2d(newX, newY, newRotation);
    }
}
