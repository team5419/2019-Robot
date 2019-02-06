package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.LockTeleOpCommand;

/**
 * Add your docs here.
 */
public class Lock extends Subsystem {
  TalonSRX lock = new TalonSRX(RobotMap.lock);

  public Lock() {
    this.lock();
  }

  public void lock() {
    lock.set(ControlMode.PercentOutput, 1);
  }

  public void unlock() {
    lock.set(ControlMode.PercentOutput, 0);
  }

  public void setLock(double percent) {
    lock.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void initDefaultCommand() {
    this.setDefaultCommand(new LockTeleOpCommand());
  }
}
