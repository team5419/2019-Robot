package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

public class ArmZeroCommand extends Command {
  public ArmZeroCommand() {
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override protected void initialize() {
    Arm.motor.set(ControlMode.PercentOutput, .3);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override protected boolean isFinished() {
    return Arm.motor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  // Called once after isFinished returns true
  @Override protected void end() {
    Arm.motor.getSensorCollection().setAnalogPosition(0, RobotMap.TimeoutMs);
    Arm.motor.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override protected void interrupted() {
    Arm.motor.set(ControlMode.PercentOutput, 0);
  }
}
