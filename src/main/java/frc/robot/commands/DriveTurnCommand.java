package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.OI;

public class DriveTurnCommand extends Command {
  private double degrees;

  public DriveTurnCommand(double degrees) {
    this.degrees = degrees;
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override protected void initialize() {
    //OI.gyro.reset();
    System.out.println("started turning!");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override protected void execute() {
    Robot.driveTrain.setMotors(0, .25, DriveTrain.DriveTrainMode.OPEN);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //double turned = Math.abs(OI.gyro.getAngle());
    //if (turned > degrees) {
    //  return true;
    //}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
