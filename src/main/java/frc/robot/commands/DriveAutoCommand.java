package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.util.NDiUnits;

public class DriveAutoCommand extends Command {
  public DriveAutoCommand() {
    requires(Robot.driveTrain);
  }

  @Override
  protected void initialize() {
    Robot.driveTrain.drive(NDiUnits.inchesToEncoderTicks(60));
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
