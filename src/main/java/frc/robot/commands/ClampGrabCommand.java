/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Clamp;

public class ClampGrabCommand extends Command {
  public enum GrabStatus {
    GRAB,
    RELEASE
  }

  GrabStatus status;

  public ClampGrabCommand(GrabStatus status) {
    this.status = status;
    requires(Robot.clamp);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (this.status == GrabStatus.GRAB) {
      Robot.clamp.grab();
    }
    if (this.status == GrabStatus.RELEASE) {
      Robot.clamp.release();
    }

    System.err.println("started");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(this.status == GrabStatus.GRAB){
      return Clamp.motor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    else if(this.status == GrabStatus.RELEASE){
      return Clamp.motor.getSensorCollection().isRevLimitSwitchClosed();
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.clamp.stop();
    System.err.println("stoped");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
