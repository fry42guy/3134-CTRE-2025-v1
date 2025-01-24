package frc.robot;

public class Constants {

    public class ElevatorConstants {
        public static final int kElevatorMotorPort = 1;
        public static final int kElevatorMotorPort2 = 2;
    }

    public class IntakeConstants {
        public static final int kIntakeMotorPort1 = 3;
        public static final int kIntakeMotorPort2 = 4;
        public static final int kIntakeMinEncoderCount = 0;
        public static final int kIntakeMaxEncoderCount = 1000;
        public static final double kIntakeIdealSpeed = 0.5;
        public static final double kIntakeDefaultP = 0.1;
        public static final double kIntakeDefaultI = 0.0;
        public static final double kIntakeDefaultD = 0.0;
    }

    public class PivotArmConstants {
        public static final int kPivotArmMotorPort = 5;
        public static final int kPivotArmMinEncoderCount = 0;
        public static final int kPivotArmMaxEncoderCount = 1000;
        public static final double kPivotArmIdealSpeed = 0.5;
        public static final double kPivotArmDefaultP = 0.1;
        public static final double kPivotArmDefaultI = 0.0;
        public static final double kPivotArmDefaultD = 0.0;
    }
    
}
