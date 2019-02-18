package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTeleOpCommand;
import frc.util.NDiUnits;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrain extends Subsystem {
  public enum DriveTrainMode {
    OPEN, CLOSED
  }

  public TalonSRX leftMotor = new TalonSRX(RobotMap.leftBackMotor);
  public VictorSPX leftMotorFollower = new VictorSPX(RobotMap.leftFrontMotor);

  public boolean reversed = false;

  public TalonSRX rightMotor = new TalonSRX(RobotMap.rightFrontMotor);
  public VictorSPX rightMotorFollower  = new VictorSPX(RobotMap.rightBackMotor);


  //https://github.com/Mercury1089/2018-robot-code/blob/master/robot/src/org/usfirst/frc/team1089/robot/commands/MoveOnPath.java
  public Trajectory.Config ProfilerConfig = new Trajectory.Config(
    Trajectory.FitMethod.HERMITE_CUBIC, // fit nethod 
    Trajectory.Config.SAMPLES_LOW, // sample count
    0.05, // time steps (m/s)
    RobotMap.driveTrainMaxVelocity, // max velocity (et/100ms)
    RobotMap.driveTrainMaxAcceleration, // max acceleration (et/100ms/100ms)
    RobotMap.driveTrainMaxAcceleration // max jerk (et/100ms/100ms/100ms)
  );

  private final SendableChooser<DriveTrainMode> modeChooser = new SendableChooser<DriveTrainMode>();

  public DriveTrain() {
    super();

    // set of leader talons
    setUpTalon(leftMotor);
    setUpTalon(rightMotor);

    // invert the right side so they go the right way
    leftMotor.setInverted(true);
    leftMotorFollower.setInverted(true);
    
    // set talon followers
    rightMotorFollower.set(ControlMode.Follower, RobotMap.rightBackMotor);
    leftMotorFollower.set(ControlMode.Follower, RobotMap.leftBackMotor);
    
    // push drive trains modes to smart dash board
    modeChooser.setDefaultOption("open", DriveTrainMode.OPEN);
    modeChooser.addOption("open", DriveTrainMode.OPEN);
    SmartDashboard.putData("drive train mode", modeChooser);
  }

  private void setUpTalon(TalonSRX talon) {
    talon.setSensorPhase(true);
		talon.configSelectedFeedbackSensor(
			FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
		);
    
    //todo: most of the values are not well configured, need to research them

		//set peak(max), nominal(min) outputs in %
		/*talon.configNominalOutputForward(0, RobotMap.TimeoutMs);
		talon.configNominalOutputReverse(0, RobotMap.TimeoutMs);
	  talon.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
		talon.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);*/
		
		//talon.selectProfileSlot(RobotMap.SlotIdx, RobotMap.PIDLoopIdx);
		talon.config_kF(RobotMap.PIDLoopIdx, RobotMap.PIDkF, RobotMap.TimeoutMs);
		talon.config_kP(RobotMap.PIDLoopIdx, RobotMap.PIDkP, RobotMap.TimeoutMs);
		talon.config_kI(RobotMap.PIDLoopIdx, RobotMap.PIDkI, RobotMap.TimeoutMs);
		talon.config_kD(RobotMap.PIDLoopIdx, RobotMap.PIDkD, RobotMap.TimeoutMs);
		
		//talon.configSelectedFeedbackSensor(0,0,0);
		talon.configMotionAcceleration(RobotMap.driveTrainMaxAcceleration, RobotMap.TimeoutMs);
		talon.configMotionCruiseVelocity(RobotMap.driveTrainMaxVelocity, RobotMap.TimeoutMs);
		
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
    double speed;
    double turn;
    if(reversed){
      speed = -OI.driverStick.getRawAxis(1);
      turn = -OI.driverStick.getRawAxis(2);
    }
    else{
      speed = OI.driverStick.getRawAxis(1);
      turn = -OI.driverStick.getRawAxis(2);
    }
    
    
    if (Math.abs(speed) < .01) {
      speed = 0;
    }

    if (Math.abs(turn) < .01) {
      turn = 0;
    }

    setMotors(speed, turn, modeChooser.getSelected());
  }

  public void stop() {
    this.setMotors(0, 0, DriveTrainMode.OPEN);
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
      double targetVelocityRight = (speed - turn) * getMaxVelocity();
      double targetVelocityLeft = (speed + turn) * getMaxVelocity();
      
      rightMotor.set(ControlMode.Velocity, targetVelocityRight); 
      leftMotor.set(ControlMode.Velocity, targetVelocityLeft);  
    }
  }

  

  public Trajectory[] pathfind(Waypoint[] points) {
    Trajectory trajectory = Pathfinder.generate(points, this.ProfilerConfig);
    TankModifier modifier = new TankModifier(trajectory);
    modifier.modify(NDiUnits.inchesToEncoderTicks(RobotMap.driveTrainWheelDistance));
    Trajectory left  = modifier.getLeftTrajectory();
    Trajectory right = modifier.getRightTrajectory();
    return new Trajectory[] {left, right};
  }

  /**
   * Dump data out onto smart dash board.
   */
  public void dump() {
    SmartDashboard.putNumber("current speed right", rightMotor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current speed left", leftMotor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current pos right", rightMotor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
		SmartDashboard.putNumber("current pos left", leftMotor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
  }

  /**
   * 
   * @param distance the distance the robot should drive.
   */
  public void drive(double distance) {
    leftMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.TimeoutMs);
    rightMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.TimeoutMs);
    leftMotor.set(ControlMode.MotionMagic, distance);
    rightMotor.set(ControlMode.MotionMagic, distance);
	}
  
  /**
   * 
   * @param distances list of distances the robot should drive
   */
	public void drive(Trajectory[] trajectorys) {
    BufferedTrajectoryPointStream leftStream = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream rightStream = new BufferedTrajectoryPointStream();

    int length = trajectorys[0].length();
    
    Trajectory.Segment leftSegment;
    Trajectory.Segment rightSegment;
    TrajectoryPoint leftPoint;
    TrajectoryPoint rightPoint;

		for (int i = 0; i < length; i++) {
      leftPoint = new TrajectoryPoint();
      rightPoint = new TrajectoryPoint();
      leftSegment = trajectorys[0].get(i);
      rightSegment = trajectorys[1].get(i);
      
			leftPoint.position = leftSegment.position;
      leftPoint.velocity = leftSegment.velocity;

			leftPoint.zeroPos = i == 0; // zero if first point
			leftPoint.isLastPoint = i + 1 == trajectorys[0].length(); // cheak if last point
      leftPoint.profileSlotSelect0 = RobotMap.PIDLoopIdx;

      rightPoint.position = rightSegment.position;
			rightPoint.velocity = rightSegment.velocity;
			rightPoint.zeroPos = i == 0; // zero if first point
			rightPoint.isLastPoint = i + 1 == trajectorys[0].length(); // cheak if last point
      rightPoint.profileSlotSelect0 = RobotMap.PIDLoopIdx;
      
      leftStream.Write(leftPoint);
      rightStream.Write(rightPoint);
    }

    leftMotor.startMotionProfile(leftStream, length, ControlMode.MotionProfile);
    rightMotor.startMotionProfile(rightStream, length, ControlMode.MotionProfile);
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

  public int getMaxVelocity() {
    if (Robot.elevator.getPosition() > RobotMap.robotSlowModeThreshold) {
      return RobotMap.slowModeMaxVelocity;
    } else {
      return RobotMap.driveTrainMaxVelocity;
    }
  }
}