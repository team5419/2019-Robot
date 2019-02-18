/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class ArmFlipCommand extends Command {
  Arm.ArmPosition position;

  public ArmFlipCommand() {
    requires(Robot.arm);
  }

  public ArmFlipCommand(Arm.ArmPosition position) {
    requires(Robot.arm);
    this.position = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if (this.position == null) {
        Robot.arm.teleOp();
      } else {
        Robot.arm.flip(position);
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
