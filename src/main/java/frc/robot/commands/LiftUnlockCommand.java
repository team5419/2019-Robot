package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftUnlockCommand extends Command {
  Timer t;

  public LiftUnlockCommand() {
    requires(Robot.lift);
    t = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    t.reset();
    t.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (t.get() > .1) {
      Robot.lift.unlock(1);
    } else {
      Robot.lift.unlock(-1);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return t.get() > .2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.stopLock();
    t.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
