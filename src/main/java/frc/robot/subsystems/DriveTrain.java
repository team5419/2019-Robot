package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
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
  public TalonSRX leftMotor;//= new TalonSRX(RobotMap.leftBackMotor);
  public TalonSRX rightMotor; //= new TalonSRX(RobotMap.rightBackMotor);
  public TalonSRX leftMotorFollower; //= new TalonSRX(RobotMap.leftFrontMotor);
  public TalonSRX rightMotorFollower; // = new TalonSRX(RobotMap.rightFrontMotor);

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
  public void teleop() {
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
   * Dumps data onto smart dash board
   */
  public void dump() {
    SmartDashboard.putNumber("current speed right", rightMotor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current speed left", leftMotor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current pos right", rightMotor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current pos left", leftMotor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
  }

  /**
   * 
   * @param distance the distance the robot should drive
   */
  public void drive(double distance) {
		drive(new double[] {distance});
	}
  
  /**
   * 
   * @param distances list of distances the robot should drive
   */
	public void drive(double[] distances) {
    BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
    
		for (int i = 0; i < distances.length; i++) {
      TrajectoryPoint point = new TrajectoryPoint();
      
			point.position = distances[i];
			point.velocity = RobotMap.maxSpeed;
			point.zeroPos = i == 0; // zero if first point
			point.isLastPoint = i + 1 == distances.length; // cheak if last point
      point.profileSlotSelect0 = RobotMap.PIDLoopIdx;
      
      stream.Write(point);
    }

    leftMotor.startMotionProfile(stream, distances.length, ControlMode.MotionProfile);
    rightMotor.startMotionProfile(stream, distances.length, ControlMode.MotionProfile);
  }

  public boolean isMotionProfileFinished() {
    return (
      rightMotor.isMotionProfileFinished() &&
      leftMotor.isMotionProfileFinished()
    );
  }

  public void stopMotionProfile() {
    rightMotor.set(ControlMode.Velocity, 0);
    leftMotor.set(ControlMode.Velocity, 0);
  }
}