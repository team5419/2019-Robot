package frc.robot.subsystems;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTeleOpCommand;

enum DriveTrainMode {
  OPEN, CLOSED
}

public class DriveTrain extends Subsystem {
  public TalonSRX leftMotor = new TalonSRX(RobotMap.leftBackMotor);
  public TalonSRX rightMotor = new TalonSRX(RobotMap.rightBackMotor);
  public TalonSRX leftMotorFollower = new TalonSRX(RobotMap.leftFrontMotor);
  public TalonSRX rightMotorFollower = new TalonSRX(RobotMap.rightFrontMotor);

  private final SendableChooser<DriveTrainMode> modeChooser = new SendableChooser<>();

  public DriveTrain() {
    super();
    
    // set of leader talons
    setUpTalon(leftMotor);
    setUpTalon(rightMotor);

    // invert the right side so they go the right way
    rightMotor.setInverted(true);
    rightMotorFollower.setInverted(true);
    
    // set talon followers
    rightMotorFollower.set(ControlMode.Follower, RobotMap.rightBackMotor);
    leftMotorFollower.set(ControlMode.Follower, RobotMap.leftBackMotor);
    
    // push drive trains modes to smart dash board
    modeChooser.setDefaultOption("open", DriveTrainMode.OPEN);
    modeChooser.addOption("open", DriveTrainMode.CLOSED);
    SmartDashboard.putData("drive train mode", modeChooser);
  }

  private void setUpTalon(TalonSRX talon) {
    talon.setSensorPhase(true);
		talon.configSelectedFeedbackSensor(
			FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
		);
    
    //todo: most of the values are not well configured, need to research them

		//set peak(max), nominal(min) outputs in %
		talon.configNominalOutputForward(0, RobotMap.TimeoutMs);
		talon.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		talon.configPeakOutputForward(1, RobotMap.TimeoutMs);
		talon.configPeakOutputReverse(-1, RobotMap.TimeoutMs);
		
		//talon.selectProfileSlot(RobotMap.SlotIdx, RobotMap.PIDLoopIdx);
		talon.config_kF(RobotMap.PIDLoopIdx, RobotMap.PIDkF, RobotMap.TimeoutMs);
		talon.config_kP(RobotMap.PIDLoopIdx, RobotMap.PIDkP, RobotMap.TimeoutMs);
		talon.config_kI(RobotMap.PIDLoopIdx, RobotMap.PIDkI, RobotMap.TimeoutMs);
		talon.config_kD(RobotMap.PIDLoopIdx, RobotMap.PIDkD, RobotMap.TimeoutMs);
		
		//talon.configSelectedFeedbackSensor(0,0,0);
		talon.configMotionAcceleration(RobotMap.Acceleration, RobotMap.TimeoutMs);
		talon.configMotionCruiseVelocity(RobotMap.maxSpeed, RobotMap.TimeoutMs);
		
		talon.configMotionProfileTrajectoryPeriod(100000,100000);
  }

  /**
   * Sets tele op command to driveTeleOpCommand
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveTeleOpCommand());
  }

  /**
   * Runs the teleOp code for the drive train
   */
  public void TeleOp() {
    double speed = -OI.driverStick.getRawAxis(1);
    double turn = OI.driverStick.getRawAxis(4);
    
    setMotors(speed, turn, modeChooser.getSelected());
  }

  /**
   * 
   * @param speed percent of max speed robot should move at
   * @param turn how much it should stray on max speed on ither side
   * @param mode wicth mode it should be in 
   */
  public void setMotors(double speed, double turn, DriveTrainMode mode) {
    if (mode == DriveTrainMode.OPEN) {
      double leftSpeed = (speed+turn);
      double rightSpeed = (speed-turn);
      leftMotor.set(ControlMode.PercentOutput, leftSpeed);
      rightMotor.set(ControlMode.PercentOutput, rightSpeed);
    } else if (mode == DriveTrainMode.CLOSED) {
      double targetVelocityRight = (speed - turn) * RobotMap.maxSpeed;
      double targetVelocityLeft = (speed + turn) * RobotMap.maxSpeed;
      
      rightMotor.set(ControlMode.Velocity, targetVelocityRight); 
      leftMotor.set(ControlMode.Velocity, targetVelocityLeft);  
    }
  }

  /**
   * called during autonomous to update the motion profile buffer
   */
  public void update() {
    rightMotor.processMotionProfileBuffer();
    leftMotor.processMotionProfileBuffer();
  }

  /**
   * Dumps data onto smart dash board
   */
  public void dump() {
    SmartDashboard.putNumber("current Speed Right", rightMotor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current Speed Left", leftMotor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current pos Right", rightMotor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current pos Left", leftMotor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
  }

  /**
   * @param distance how far the robot should drive
   */
  public void drive(double distance) {
    //1.0 keeps streaming points, once at set position goes to next point
		//2.0 stops it at whatever point is at right now
		leftMotor.set(ControlMode.MotionProfile, 1.0);
    rightMotor.set(ControlMode.MotionProfile, 1.0);
    
    TrajectoryPoint point = new TrajectoryPoint();
		point.position = distance;
		point.velocity = RobotMap.maxSpeed;
		point.zeroPos = true;
		point.isLastPoint = true;
    point.profileSlotSelect0 = RobotMap.PIDLoopIdx;
    
    leftMotor.pushMotionProfileTrajectory(point);
    rightMotor.pushMotionProfileTrajectory(point);
  }
}