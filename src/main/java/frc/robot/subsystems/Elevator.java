package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorAutoCommand;

enum ElevatorPosition {
  UP, DOWN
}

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  public TalonSRX motor = new TalonSRX(RobotMap.elevatorMotor);

  public Elevator() {
    // set up talon
  }

  /**
   * Sets tele op command to driveTeleOpCommand
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorAutoCommand());
  }

  public void teleop() {

  }

  public void positon(ElevatorPosition position) {
    if (position == ElevatorPosition.UP) {
      motor.set(ControlMode.Position, 0);
    } else if (position == ElevatorPosition.UP) {
      motor.set(ControlMode.Position, RobotMap.elevatorMaxPosition);
    }
  }

  public void isPositon(ElevatorPosition position) {
    
  }
}