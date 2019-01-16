package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ElevatorAutoCommand;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  
  /**
   * Sets tele op command to driveTeleOpCommand
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorAutoCommand());
  }

  public void teleop() {

  }
}
