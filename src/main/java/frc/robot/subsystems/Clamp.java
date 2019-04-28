package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Clamp extends Subsystem {
  public static TalonSRX motor = new TalonSRX(RobotMap.clamp);
  public static boolean isGrab = false;
  public static DigitalInput openLimit, closeLimit;

  public Clamp() {
    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
    motor.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);
	motor.setSensorPhase(false);

    openLimit = new DigitalInput(RobotMap.openLimitSwitch);
    closeLimit = new DigitalInput(RobotMap.closeLimitSwitch);
    
    ConfigMotor(motor);
  }

  public void teleOp() {
    /*double shift = OI.operatorStick.getRawAxis(0);
    if (Math.abs(shift) < .2) {
      shift = shift / 5;
      motor.set(ControlMode.PercentOutput, shift);
    }*/
  }

  private void ConfigMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative,
      RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
    );
    
    /* set allowed voltage */
    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(1, RobotMap.TimeoutMs);
		motor.configPeakOutputReverse(-1, RobotMap.TimeoutMs);
    
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

  @Override public void initDefaultCommand() {
    //setDefaultCommand(new ClampTeleOpCommand());
  }

  public void grab() {
    //motor.set(ControlMode.Position, 0);
    motor.set(ControlMode.PercentOutput, .3);
    isGrab = true;
  }

  public void release() {
    //motor.set(ControlMode.Position, 10);
    motor.set(ControlMode.PercentOutput, -.25);
    isGrab = false;
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void dump() {
    SmartDashboard.putBoolean("open limit", openLimit.get());
    SmartDashboard.putBoolean("close limit", closeLimit.get());

    SmartDashboard.putNumber("clamp current", motor.getOutputCurrent());
  }
}
