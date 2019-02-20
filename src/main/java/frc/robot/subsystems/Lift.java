package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.LiftTeleOpCommand;

public class Lift extends Subsystem {
  TalonSRX lock = new TalonSRX(RobotMap.lock);
  TalonSRX liftMotor = new TalonSRX(RobotMap.rightLiftMotor);
  TalonSRX liftMotorFallower = new TalonSRX(RobotMap.leftLiftMotor);
  public boolean locked = true;

  public Lift() {
    // set up lock

    lock.configNominalOutputForward(0, RobotMap.TimeoutMs);
		lock.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		lock.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
    lock.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);

    //this.lock();

    // set up lift
    
    ConfigMotor(liftMotor);
    liftMotorFallower.set(ControlMode.Follower, liftMotor.getDeviceID());
  }

  public void unlock() {
    lock.set(ControlMode.PercentOutput, .1);
    this.locked = false;
  }

  public void stopLock() {
    lock.set(ControlMode.PercentOutput, .1);
    this.locked = false;
  }

  @Override public void initDefaultCommand() {
    this.setDefaultCommand(new LiftTeleOpCommand());
  }

  public void teleOp() {
      // double percentOutput = OI.operatorStick.getRawAxis(3) / 2;
      // liftMotor.set(ControlMode.PercentOutput, percentOutput);
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
		motor.config_kP(RobotMap.PIDLoopIdx, RobotMap.PIDkP, RobotMap.TimeoutMs);
		motor.config_kI(RobotMap.PIDLoopIdx, RobotMap.PIDkI, RobotMap.TimeoutMs);
    motor.config_kD(RobotMap.PIDLoopIdx, RobotMap.PIDkD, RobotMap.TimeoutMs);

    motor.configMotionAcceleration(1000, RobotMap.TimeoutMs);
    motor.configMotionCruiseVelocity(100, RobotMap.TimeoutMs);
  }

  public void lift() {
    liftMotor.set(ControlMode.PercentOutput, .75);
  }

  public void stop() {
    liftMotor.set(ControlMode.PercentOutput, 0);
  }
}
