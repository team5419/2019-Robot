package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveAutoCommand extends Command {
  public DriveAutoCommand() {
    requires(Robot.driveTrain);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
  }

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
