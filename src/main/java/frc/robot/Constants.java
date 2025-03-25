package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {

public static final boolean usecameratoupdatepose = true;


    public class coralv2{

        public static final double Drivekp = 1.2;
        public static final double Driveki = .01;
        public static final double Drivekd = 0.001;
        public static final double drivedeadband = .3;

        public static final double yDrivekp = 1.2;
        public static final double yDriveki = .010;
        public static final double yDrivekd = 0.001;
        



        public static final double Rotatekp = 4; //5
        public static final double Rotateki = .1; //0
        public static final double Rotatekd = .001; //.001
        public static final double rotatedeadband = .1; //.1

        public static final double DriveMaxSpeed = 1;
        public static final double RotateMaxSpeed = 3; //3

        public static final double leftxtarget = -.5; // tz in limelight (meters)
        public static final double leftytarget = 0.0; //tx in limelight (meters)
        public static final double leftztarget = 0.0; //RY in limelight (degrees)

        public static final double rightxtarget = -0.5; // tx in limelight (meters)
        public static final double rightytarget = 0.0;    //TZ in limelight (meters)
        public static final double rightztarget = 0; //RY in limelight (degrees)

        public static final double Xtolerance = 0.01;
        public static final double Ytolerance = 0.01;
        public static final double Ztolerance = 0.0001;



        
        




    }


public class LimeLightConstants {

    public static final double Side_to_side_offset_in = 6.46875;
    public static final double Left_offset = 6.46875;
    public static final double Right_offset = 9.46875;
    public static final double Front_to_back_offset_in = 18;

}


    public class ElevatorConstants {
        public static final double home = .2;
        public static final double bottomrung = 10.23;
        public static final double middlerung = 27.74;
        public static final double toprung = 56.92;
        public static final double loweralge =22.4;
        public static final double upperalge = 37.7;
        public static final double processalge = 5;//??






        public static final int kElevatorMotorPort = 9;
        public static final int kElevatorMotorPort2 = 10;
        public static final double ElevatorMotorMinSoftLimit = 0;
        public static final double ElevatorMotorMaxSoftLimit = 57; //comfimed 56 is max
 
        public static final InvertedValue kElevatorMotor1Inverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kElevatorMotor2Inverted = InvertedValue.Clockwise_Positive;
        public static final double homespeed = -.2;
        public static final double testspeed = .50;

public static final double Setpoint1 = 10; // Setpoint 1 for the elevator frostw/h
public static final double Setpoint2 = 20; // Setpoint 2 for the elevator
public static final double Setpoint3 = 30; // Setpoint 3 for the elevator
//public static final double Setpoint4 = 0; // Setpoint 4 for the elevator /////moved to shouffel board

public static final double Elevatorkp = 2.4; // An error of 1 rotation results in 2.4 V output
public static final double Elevatorki = 0; // No output for integrated error
public static final double Elevatorkd = 0.1; // A velocity of 1 rps results in 0.1 V output

public static final double PeakForwardVoltage = 4.8; // Peak output of 4 V   ***********************************Default was 8...4?
public static final double PeakReverseVoltage = -2.5 ; // Peak output of -1 V ***********************************Default was -8



    }

    public class IntakeConstants {
        public static final int kIntakeMotorPort1 = 11;
        public static final int kIntakeMotorPort2 = 12;
        public static final int kCanRageID = 14;
        public static final double CanRangeDetectDistance = .25; // Set the distance you want to detect objects at in Meters
        public static final double REVspeed = -0.5; // Example value, set to your desired reverse speed
        public static final double FWDspeed = .5;  // Example value, set to your desired forward speed
        // public static final int kIntakeMinEncoderCount = 0;
        // public static final int kIntakeMaxEncoderCount = 1000;
        // public static final double kIntakeIdealSpeed = 0.5;
        // public static final double kIntakeDefaultP = 0.1;
        // public static final double kIntakeDefaultI = 0.0;
        // public static final double kIntakeDefaultD = 0.0;
        public static final InvertedValue kIntakeMotor1Inverted = InvertedValue.Clockwise_Positive;
        //public static final InvertedValue kIntakeMotor2Inverted = InvertedValue.Clockwise_Positive;
    }

    public class PivotArmConstants {

        

        public static final double home = 0*.625;
        public static final double bottomrung = 4.9*.625;
        public static final double middlerung = 4.7*.625;
        public static final double toprung = 8.9*.625;
        public static final double loweralge =32.2*.625;
        public static final double upperalge = 28.5*.625;
        public static final double processalge = 25;//?





        public static final int kPivotArmMotorPort = 13;
        public static final double PivotArmMotorMaxSoftLimit = 33*.625; // Define the max soft limit
        public static final double PivotArmMotorMinSoftLimit = 0; // Define the min soft limit
        public static final double homespeed = -.05;
        public static final double testspeed = .15;
        public static final InvertedValue kPivotArmMotorInverted = InvertedValue.Clockwise_Positive;//InvertedValue.Clockwise_Positive;
        public static final int kPivotArmMinEncoderCount = 0;
        public static final int kPivotArmMaxEncoderCount = 25;
        //public static final double kPivotArmIdealSpeed = 0.5;
        public static final double kPivotArmkP = 2.4;
        public static final double kPivotArmkI = 0.0;
        public static final double kPivotArmkD = 0.1;

        public static final double Setpoint1 = 5*.625; // Setpoint 1 for the elevator
        public static final double Setpoint2 = 10*.625; // Setpoint 2 for the elevator
        public static final double Setpoint3 = 15*.625; // Setpoint 3 for the elevator


        public static final double PeakForwardVoltage = 4; // Peak output of 4 V   ***********************************Default was 8...4?
public static final double PeakReverseVoltage = -4 ; // Peak output of -1 V ***********************************Default was -8




    }


    public class ClimberConstants{

        public static final int kClimberMotorPort = 15;
        public static final double kfwdspeed = 0.85;
        public static final double krevspeed = -0.85;
        public static final InvertedValue kClimberMotorInverted =InvertedValue.Clockwise_Positive;


    }

    public class ApriltagConstants {
       public final double coral_Leftrightoffset_IN = 12;
         public final double coral_Frontbackoffset_IN = 12;
            public final double coral_Rotationoffset_deg= 0;
    }
    
}
