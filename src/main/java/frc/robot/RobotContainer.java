// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Commands.ElevatorPIDSetpoint;
import frc.robot.subsystems.*;

public class RobotContainer {

private final ElevatorSubsytem m_elevator = new ElevatorSubsytem();
private final IntakeSubsystem m_intake = new IntakeSubsystem();
private final PivotArmSubsystem m_pivotArm = new PivotArmSubsystem();













    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
m_intake.setDefaultCommand(new RunCommand(() -> m_intake.Setspeed(0.0), m_intake));
m_pivotArm.setDefaultCommand(new RunCommand(() -> m_pivotArm.Setspeed(0.0), m_pivotArm));
m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.Setspeed(0.0), m_elevator));

      

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.a().onTrue(new ElevatorPIDSetpoint(m_elevator , Constants.ElevatorConstants.Setpoint1));
        joystick.b().onTrue(new ElevatorPIDSetpoint(m_elevator , Constants.ElevatorConstants.Setpoint2));
        joystick.x().onTrue(new ElevatorPIDSetpoint(m_elevator , Constants.ElevatorConstants.Setpoint3));
        //joystick.y().onTrue(new PIDSetpoint(m_elevator , SmartDashboard.getNumber("TestSetpoint", 100)));
        joystick.rightBumper().onTrue(m_elevator.runOnce(() -> m_elevator.BreakModeOn(true)));
        joystick.leftBumper().onTrue(m_elevator.runOnce(() -> m_elevator.BreakModeOn(false)));
        joystick.rightTrigger().whileTrue(new RunCommand(() -> m_intake.Setspeed(Constants.IntakeConstants.REVspeed), m_intake));
        joystick.leftTrigger().whileTrue(new RunCommand(() -> m_intake.Setspeed(Constants.IntakeConstants.FWDspeed), m_intake));



        joystick2.leftBumper().onTrue(m_elevator.runOnce(() -> m_elevator.softlimitsOFF()));
        joystick2.rightBumper().onTrue(m_elevator.runOnce(() -> m_elevator.softlimitsOn()));
        joystick2.leftTrigger().whileTrue(new RunCommand(() -> m_elevator.Setspeed(Constants.ElevatorConstants.testspeed), m_elevator));
        joystick2.rightTrigger().whileTrue(new RunCommand(() -> m_elevator.Setspeed(-Constants.ElevatorConstants.testspeed), m_elevator));
        
        joystick2.a().onTrue(m_pivotArm.runOnce(()-> m_pivotArm.softlimitsOFF()));
        joystick2.b().onTrue(m_pivotArm.runOnce(()-> m_pivotArm.softlimitsOn()));
        joystick2.x().onTrue(new RunCommand(() -> m_pivotArm.Setspeed(Constants.PivotArmConstants.testspeed), m_pivotArm));
        joystick2.y().onTrue(new RunCommand(() -> m_pivotArm.Setspeed(-Constants.PivotArmConstants.testspeed), m_pivotArm));


        

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
