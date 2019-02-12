package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.LiftTeleOpCommand;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  TalonSRX lock = new TalonSRX(RobotMap.lock);
  TalonSRX liftMotor = new TalonSRX(RobotMap.rightLiftMotor);
  TalonSRX liftMotorFallower = new TalonSRX(RobotMap.leftLiftMotor);
  public boolean locked = false;

  public Lift() {
    // set up lock

    lock.configNominalOutputForward(0, RobotMap.TimeoutMs);
		lock.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		lock.configPeakOutputForward(1, RobotMap.TimeoutMs);
    lock.configPeakOutputReverse(-1, RobotMap.TimeoutMs);

    this.lock();

    // set up lift
    
    liftMotorFallower.set(ControlMode.Follower, liftMotor.getDeviceID());
  }

  public void teleOp() {

  }

  public void lock() {
    lock.set(ControlMode.PercentOutput, 1);
    this.locked = true;
  }

  public void unlock() {
    lock.set(ControlMode.PercentOutput, 0);
    this.locked = false;
  }

  @Override
  public void initDefaultCommand() {
    this.setDefaultCommand(new LiftTeleOpCommand());
  }
}
