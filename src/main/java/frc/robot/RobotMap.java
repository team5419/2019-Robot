package frc.robot;

/**
 * Robot wide constants
 */
public class RobotMap {
    // drive train motion configuration
    public static int driveTrainMaxVelocity = 5189; // (et / 100ms)
    public static int driveTrainMaxAcceleration = 1000; // (et / 100ms / 100ms)
    public static int driveTrainMaxJerk = 1000; // (et / 100ms / 100ms / 100ms)

    public static double driveTrainWheelDistance = 10; // (inch)
    public static double driveTrainWheelDiameter = 6; // (inch)
    public static double driveTrainWheelCircumference = 2 * Math.PI * driveTrainWheelDiameter; // (inch)
    public static double driveTrainTicksPerRevelution =  360; //

    // elevator motion configuration
    public static double elevatorMaxPosition = -50000; // (et)
    public static double robotSlowModeThreshold = 100; // (et)
    public static int slowModeMaxVelocity = 100;

    // Limit Switch Ports
    public static int openLimitSwitch = 1;
    public static int closeLimitSwitch = 0;

    // ids of talonSRXs
    public static int leftBackMotor = 10;
    public static int leftFrontMotor = 8;
    public static int rightBackMotor = 12;
    public static int rightFrontMotor = 9;

    public static int leftElevatorMotor = 5;
    public static int rightElevatorMotor = 3;   

    public static int leftLiftMotor = 11;
    public static int rightLiftMotor = 4;

    public static int lock = 7;

    public static int clamp = 2;
    public static int arm = 6;

    // PID loop constants
    public static int PIDLoopIdx = 0;
    public static double PIDkF = 0.4;
    public static double PIDkP = 0.5;
    public static double PIDkI = 0;
    public static double PIDkD = 0;

    // misc
    public static int TimeoutMs = 0;
    public static double percent = 1;
}