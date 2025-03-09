package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

     private final Field2d m_feild = new Field2d();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private AprilTagFieldLayout  fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public Integer lastTagID = -2;
    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        
    }


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {

       

        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public int convertIDtoInt(double tagdouble ){
        Integer tagint = (int) tagdouble;
     
         return tagint;
     
      }
     
 
    public void periodic() {



      //  System.out.println(getTagID().getAsInt());

        if (Utils.isSimulation()){
           
            lastTagID = convertIDtoInt(SmartDashboard.getNumber("lastTagID", -1.0));
          
          }
          else{
            lastTagID = convertIDtoInt(LimelightHelpers.getFiducialID("limelight"));
          
          }
          


        SmartDashboard.putNumber("April ID", LimelightHelpers.getFiducialID("limelight"));
       


if (DriverStation.isTeleop()) {
Boolean doRejectUpdate = false;
    LimelightHelpers.PoseEstimate  mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
   
   if (mt1 != null){
    if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    {
      if(mt1.rawFiducials[0].ambiguity > .7)
      {
        doRejectUpdate = true;
      }
      if(mt1.rawFiducials[0].distToCamera > 3)
      {
        doRejectUpdate = true;
      }
    }
    if(mt1.tagCount == 0)
    {
      doRejectUpdate = true;
    }

    if(!doRejectUpdate)
      {
        addVisionMeasurement( mt1.pose ,mt1.timestampSeconds);
      }
    }
    
}



m_feild.setRobotPose(getState().Pose);
        SmartDashboard.putData("Feild", m_feild);


       // SmartDashboard.putNumber("Camera_Target_Rotation", Units.radiansToDegrees(LimelightHelpers.getBotPose3d_TargetSpace("limelight")));
        SmartDashboard.putNumber("Camera_Target_x", Units.metersToInches(LimelightHelpers.getBotPose3d_TargetSpace("limelight").getX()));
        SmartDashboard.putNumber("Camera_Target_y", Units.metersToInches(LimelightHelpers.getBotPose3d_TargetSpace("limelight").getZ()));


        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }


    public Optional<Pose3d> getTagPose(int tagID) {
        return fieldLayout.getTagPose(tagID);
    }



//  public  Pose2d getTargetPose() {





//         // Convert AprilTag 3D Pose to 2D (X, Y, and Rotation)
//         //Pose2d tagPose2d = tagPose.toPose2d();
//         Pose2d tagPose2d = fieldLayout.getTagPose(8).get().toPose2d();
//         Rotation2d tagRotation = tagPose2d.getRotation();

//         // Define offset (move 1 foot left, 1 foot back)
//         double offsetX = Units.feetToMeters(1.0); // Left relative to tag
//         double offsetY = Units.feetToMeters(1.0); // Back relative to tag

//         // Transform offset based on tag's rotation
//         Transform2d transform = new Transform2d(new Translation2d(offsetX, offsetY), tagRotation);

//         // Compute the new pose
//         return tagPose2d.plus(transform);
//     }

// public Pose2d getTargetPose() {
//     // Get the tag pose in field coordinates





//     Optional<Pose3d> tagPose3dOpt = fieldLayout.getTagPose(17);
//     if (tagPose3dOpt.isEmpty()) {
//         return new Pose2d(); // Return a default pose if tag is not found
//     }

//     Pose2d tagPose2d = tagPose3dOpt.get().toPose2d();
//     Rotation2d tagRotation = tagPose2d.getRotation(); // Tag's rotation on the field

//     // Define offset (1 foot LEFT relative to the AprilTag's facing direction)
//     Translation2d offset = new Translation2d(
//         1,                     // No forward/backward movement
//       1 // Units.feetToMeters(1) // Move LEFT in the AprilTag's local frame
//     );

//     // Rotate the offset to align with the tag’s orientation
//     Translation2d transformedOffset = offset.rotateBy(tagRotation.unaryMinus()); 

//     // Compute the new pose (final field coordinates)
//     Pose2d targetPose = new Pose2d(
//         tagPose2d.getTranslation().plus(transformedOffset),
//         tagRotation // Keep robot facing the tag
//     );

//   SmartDashboard.putNumber("TargetPoseX", targetPose.getX());
//     SmartDashboard.putNumber("TargetPoseY", targetPose.getY());
//     SmartDashboard.putNumber("TargetPoseRotation", targetPose.getRotation().getRadians());
//     SmartDashboard.putNumber("TagPosX", tagPose2d.getX());
//     SmartDashboard.putNumber("TagPosY", tagPose2d.getY());
//     SmartDashboard.putNumber("TagRotation", tagPose2d.getRotation().getRadians());


//     return targetPose;
// }

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
    double leftOffset = Units.inchesToMeters(Constants.LimeLightConstants.Side_to_side_offset_in);
    double backwardOffset = Units.inchesToMeters(-Constants.LimeLightConstants.Front_to_back_offset_in);

    if (!Leftside) {
        leftOffset = leftOffset*-1;
    }

    // Calculate the new position
    double newX = tagX - (backwardOffset * Math.cos(tagTheta)) + (leftOffset * Math.sin(tagTheta));
    double newY = tagY - (backwardOffset * Math.sin(tagTheta)) - (leftOffset * Math.cos(tagTheta));

    // The robot should face the same direction as the tag
    Rotation2d newRotation = tagPose.getRotation();//.plus(new Rotation2d(Units.degreesToRadians(180)));

    // Return the new pose
    return new Pose2d(newX, newY, newRotation);
}




// public int getTagID() {

// double d_id = LimelightHelpers.getFiducialID("limelight");

// int id = (int) d_id;

//     return id;

// }

public IntSupplier getTagID() {

    IntSupplier tagID = () -> lastTagID;

    return tagID;
}


public void driveToCoral2(Boolean leftSide) {

    PathConstraints constraints = new PathConstraints(
        3, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands

    //double d_ID = LimelightHelpers.getFiducialID("limelight");

    //int ID = getTagID();

   SmartDashboard.putNumber("April ID_2", lastTagID);


   if (lastTagID > 0){
     AutoBuilder.pathfindToPose(
       // pose,
       getTargetPose(lastTagID, leftSide),
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
}

}

public int getlastTagID() {
    return lastTagID;
}




public Command Autodrive( Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity){



    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        goalEndVelocity
    );
}






 public Command driveToCoral(IntSupplier ID, Boolean leftSide) {
    // Create the constraints to use while pathfinding








    PathConstraints constraints = new PathConstraints(
        3, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands

    //double d_ID = LimelightHelpers.getFiducialID("limelight");

    //int ID = getTagID();

   //SmartDashboard.putNumber("April ID_2", lastTagID);

 


if (ID.getAsInt() > 0){
    return AutoBuilder.pathfindToPose(
       // pose,
       getTargetPose(ID.getAsInt(), leftSide),
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
}

else {
    return dothingCommand();



  }
 }

  public Command dothingCommand() {
    System.out.println("Do nothing here");
    return run(() -> {
        // Do something here
    });









  }
}
