package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorTeleOpCommand extends Command {
  Elevator.ElevatorPosition position;
  public ElevatorTeleOpCommand(Elevator.ElevatorPosition position) {
    requires(Robot.elevator);
    this.position = position;
  }

  public ElevatorTeleOpCommand() {
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (this.position != null) {
      Robot.elevator.goToPositon(position);
    } else {
      Robot.elevator.teleop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return this.position != null;
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
