package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriverFlipDirectionCommand extends Command {
  public DriverFlipDirectionCommand() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.reversed = !Robot.driveTrain.reversed;
    /*if (Robot.driveTrain.reversed) {
      CameraServer.getInstance().getServer().setSource(Robot.frontCamera);
    } else {
      CameraServer.getInstance().getServer().setSource(Robot.backCamera);
    }*/
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
