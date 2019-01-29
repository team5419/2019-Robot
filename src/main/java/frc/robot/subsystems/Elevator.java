package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorTeleOpCommand;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  public enum ElevatorPosition {
    UP, DOWN
  }

  public static TalonSRX motor = new TalonSRX(RobotMap.leftElevatorMotor);
  public static TalonSRX motorFollower = new TalonSRX(RobotMap.rightElevatorMotor);

  public Elevator() {
    ConfigMotor(motor);
    motor.setSensorPhase(true);
    motor.setInverted(true);

    motorFollower.set(ControlMode.Follower, RobotMap.leftElevatorMotor);
    motorFollower.setSensorPhase(false);
    motorFollower.setInverted(false);
  }

  private void ConfigMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative,
      RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
    );
    
    /* set allowed voltage */
    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(1, RobotMap.TimeoutMs);
		motor.configPeakOutputReverse(-1, RobotMap.TimeoutMs);
    
    /* how wrong motor is allowed to be */
    motor.configAllowableClosedloopError(0, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);

    /* set variables for PID loops */
    motor.config_kF(RobotMap.PIDLoopIdx, RobotMap.PIDkF, RobotMap.TimeoutMs);
		motor.config_kP(RobotMap.PIDLoopIdx, RobotMap.PIDkP, RobotMap.TimeoutMs);
		motor.config_kI(RobotMap.PIDLoopIdx, RobotMap.PIDkI, RobotMap.TimeoutMs);
    motor.config_kD(RobotMap.PIDLoopIdx, RobotMap.PIDkD, RobotMap.TimeoutMs);

    motor.configMotionAcceleration(1000, RobotMap.TimeoutMs);
    motor.configMotionCruiseVelocity(100, RobotMap.TimeoutMs);
  }

  /**
   * Sets tele op command to driveTeleOpCommand
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorTeleOpCommand());
  }

  public void teleop() {
    double percentOutput = -OI.driverStick.getRawAxis(5);
    SmartDashboard.putNumber("percentOutput", percentOutput);
    motor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void goToPositon(ElevatorPosition position) {
    if (position == ElevatorPosition.DOWN) {
      System.out.println("DOWN");
      motor.set(ControlMode.MotionMagic, 0);
    } else if (position == ElevatorPosition.UP) {
      System.out.println("UP");
      motor.set(ControlMode.MotionMagic, RobotMap.elevatorMaxPosition);
    }
  }

  public boolean isPositon(ElevatorPosition position) {
    int pos = motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx);
    if (position == ElevatorPosition.DOWN) {
      return pos == 0;
    } else if (position == ElevatorPosition.UP) {
      return pos == RobotMap.elevatorMaxPosition;
    } else {
      return false;
    }
  }

  public void dump() {
    SmartDashboard.putNumber("current elevator position", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
    SmartDashboard.putNumber("current elevator velocity", motor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
  }
}