package frc.robot;

/**
 * Robot wide constants
 */
public class RobotMap {
    // motion configuration
    public static int driveTrainMaxVelocity = 5189; // (su / 100ms)
    public static int driveTrainMaxAcceleration = 1000; // (su / 100ms / 100ms)
    public static int driveTrainMaxJerk = 1000; // (su / 100ms / 100ms / 100ms)

    public static double driveTrainWheelDistance = 10; // (meters)
    public static double driveTrainWheelCircumference = -1; // 
    public static double driveTrainTicksPerRevelution = -1; // 

    public static double elevatorMaxPosition = 1000; // (su)

    // ids of talonSRXs
    public static int leftBackMotor = -1; // BS NUMBER!
    public static int rightBackMotor = -1; // BS NUMBER!
    public static int leftFrontMotor = -1; // BS NUMBER!
    public static int rightFrontMotor = -1; // BS NUMBER!
    
    public static int leftElevatorMotor = 0;
    public static int rightElevatorMotor = 6;

    // PID loop constants
    public static int PIDLoopIdx = 0;
    public static double PIDkF = 0.1989;
    public static double PIDkP = 0.2779895;
    public static double PIDkI = 0;
    public static double PIDkD = 0;

    // misc
    public static int TimeoutMs = 0;
}