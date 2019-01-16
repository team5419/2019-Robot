package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveAutoCommand extends Command {
  public DriveAutoCommand() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveTrain.isMotionProfileFinished();
  }

  @Override
  protected void end() {
    Robot.driveTrain.stopMotionProfile();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
