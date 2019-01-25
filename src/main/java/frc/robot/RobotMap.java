package frc.robot;

/**
 * Robot wide constants
 */
public class RobotMap {
    // motion configuration
    public static int maxSpeed = 5189; // sensor units / 100ms
    public static int Acceleration = 1000; // sensor units / 100ms / sec
    public static int elevatorMaxPosition = 1000; // sensor units

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