package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ClampTeleOpCommand;

/**
 * Add your docs here.
 */
public class Clamp extends Subsystem {
  public static TalonSRX motor = new TalonSRX(72);
  public static boolean isGrab = false;
  //public static DigitalInput openLimit, closeLimit;

  public Clamp() {
    motor.setInverted(true);

    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
    motor.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);

    ConfigMotor(motor);
  }

  public void teleOp() {
    if (motor.getSensorCollection().isRevLimitSwitchClosed()) {
      isGrab = true;
    }

    
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
    motor.set(ControlMode.PercentOutput, -.35);
    isGrab = true;
  }

  public void release() {
    //motor.set(ControlMode.PercentOutput, .25);
    isGrab = false;
  }

  public void stop() {
    //motor.set(ControlMode.PercentOutput, 0);
  }

  public void dump() {
    //SmartDashboard.putBoolean("open limit", openLimit.get());
    //SmartDashboard.putBoolean("close limit", closeLimit.get());

    SmartDashboard.putNumber("clamp current", motor.getOutputCurrent());
  }
}
