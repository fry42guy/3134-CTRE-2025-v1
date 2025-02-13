package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {

    public class ElevatorConstants {
        public static final int kElevatorMotorPort = 9;
        public static final int kElevatorMotorPort2 = 10;
        public static final double ElevatorMotorMinSoftLimit = 0;
        public static final double ElevatorMotorMaxSoftLimit = 57; //comfimed 56 is max
 
        public static final InvertedValue kElevatorMotor1Inverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kElevatorMotor2Inverted = InvertedValue.Clockwise_Positive;
        public static final double homespeed = -.05;
        public static final double testspeed = .20;

public static final double Setpoint1 = 10; // Setpoint 1 for the elevator
public static final double Setpoint2 = 20; // Setpoint 2 for the elevator
public static final double Setpoint3 = 30; // Setpoint 3 for the elevator
//public static final double Setpoint4 = 0; // Setpoint 4 for the elevator /////moved to shouffel board

public static final double Elevatorkp = 2.4; // An error of 1 rotation results in 2.4 V output
public static final double Elevatorki = 0; // No output for integrated error
public static final double Elevatorkd = 0.1; // A velocity of 1 rps results in 0.1 V output

public static final double PeakForwardVoltage = 5; // Peak output of 4 V   ***********************************Default was 8...4?
public static final double PeakReverseVoltage = -1 ; // Peak output of -1 V ***********************************Default was -8



    }

    public class IntakeConstants {
        public static final int kIntakeMotorPort1 = 11;
        public static final int kIntakeMotorPort2 = 12;
        public static final double REVspeed = -0.25; // Example value, set to your desired reverse speed
        public static final double FWDspeed = 0.25;  // Example value, set to your desired forward speed
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
        public static final int kPivotArmMotorPort = 13;
        public static final double PivotArmMotorMaxSoftLimit = 25; // Define the max soft limit
        public static final double PivotArmMotorMinSoftLimit = 0; // Define the min soft limit
        public static final double homespeed = -.05;
        public static final double testspeed = .15;
        public static final InvertedValue kPivotArmMotorInverted = InvertedValue.Clockwise_Positive;
        public static final int kPivotArmMinEncoderCount = 0;
        public static final int kPivotArmMaxEncoderCount = 25;
        //public static final double kPivotArmIdealSpeed = 0.5;
        public static final double kPivotArmkP = 2.4;
        public static final double kPivotArmkI = 0.0;
        public static final double kPivotArmkD = 0.1;

        public static final double Setpoint1 = 5; // Setpoint 1 for the elevator
        public static final double Setpoint2 = 10; // Setpoint 2 for the elevator
        public static final double Setpoint3 = 15; // Setpoint 3 for the elevator


        public static final double PeakForwardVoltage = 2; // Peak output of 4 V   ***********************************Default was 8...4?
public static final double PeakReverseVoltage = -1 ; // Peak output of -1 V ***********************************Default was -8




    }
    
}
