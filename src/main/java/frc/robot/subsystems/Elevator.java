package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorTeleOpCommand;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  public enum ElevatorPosition {
    UP, DOWN, MID
  }

  public static TalonSRX motorFollower = new TalonSRX(RobotMap.leftElevatorMotor);
  public static TalonSRX motor = new TalonSRX(RobotMap.rightElevatorMotor);
  // public static TalonSRX motor = new TalonSRX(RobotMap.arm);

  public int target = -500;

  public Elevator() {
    motorFollower.follow(motor);
    motorFollower.setInverted(true);

    ConfigMotor(motor);

    /* set allowed voltage */
    motorFollower.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motorFollower.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motorFollower.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
		motorFollower.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);

    motor.setSensorPhase(false);
    motor.setInverted(true);

    // Reset Encoder
    motor.setSelectedSensorPosition(0);
  }

  private void ConfigMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative,
      RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
    );

    
    /* set allowed voltage */
    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
		motor.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);
    
    /* how wrong motor is allowed to be */
    motor.configAllowableClosedloopError(0, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);

    /* set variables for PID loops */
    motor.config_kF(RobotMap.PIDLoopIdx, RobotMap.PIDkF, RobotMap.TimeoutMs);
		motor.config_kP(RobotMap.PIDLoopIdx, .75, RobotMap.TimeoutMs);
	  motor.config_kI(RobotMap.PIDLoopIdx, 0, RobotMap.TimeoutMs);
    motor.config_kD(RobotMap.PIDLoopIdx, 3, RobotMap.TimeoutMs);

    motor.configMotionAcceleration(4000, RobotMap.TimeoutMs);
    motor.configMotionCruiseVelocity(15000, RobotMap.TimeoutMs);
  }

  /**
   * Sets tele op command to driveTeleOpCommand
   */
  @Override public void initDefaultCommand() {
    setDefaultCommand(new ElevatorTeleOpCommand());
  }

  public void teleop() {
    double pos = OI.operatorStick.getRawAxis(1); // 5
    if (Math.abs(pos) < .1) {
      pos = 0;
    }
    target += pos * 100;
    if (target > -500) {
      target = -500;
    }
    if (target < -49000) {
      target = -49000;
    }

    if (Robot.arm.target == 0 && Arm.motor.getSelectedSensorPosition() < 10) {

    }
    motor.set(ControlMode.MotionMagic, target);
  }

  public void goToPositon(ElevatorPosition position) {
    if (position == ElevatorPosition.DOWN) {
      System.err.println("DOWN");
      target = -500;
      motor.set(ControlMode.MotionMagic, -500);
    } else if (position == ElevatorPosition.UP) {
      int up_pos = -49000;//-49000;
      target = up_pos;
      motor.set(ControlMode.MotionMagic, up_pos);
    } else if(position == ElevatorPosition.MID){
      int mid_pos = -26000;//-26000;
      target = mid_pos;
      motor.set(ControlMode.MotionMagic, mid_pos);
    }
  }

  public double getPosition() {
    return motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx);
  }

  public boolean isPositon(ElevatorPosition position) {
    if (position == ElevatorPosition.DOWN) {
      return getPosition() == 0;
    } else if (position == ElevatorPosition.UP) {
      return getPosition() == RobotMap.elevatorMaxPosition;
    } else {
      return false;
    }
  }

  public void dump() {
    SmartDashboard.putNumber("current elevator position", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
    SmartDashboard.putNumber("current elevator velocity", motor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
    SmartDashboard.putNumber("elevator current", motor.getOutputCurrent());
    SmartDashboard.putNumber("elevator helper current", motorFollower.getOutputCurrent());
  }
}