package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorAutoCommand extends Command {
  public ElevatorAutoCommand() {
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.goToPositon(Elevator.ElevatorPosition.UP);  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
    end();
  }
}
