package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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

  public static TalonSRX motorFollower = new TalonSRX(RobotMap.leftElevatorMotor);
  public static TalonSRX motor = new TalonSRX(RobotMap.rightElevatorMotor);
  // public static TalonSRX motor = new TalonSRX(RobotMap.arm);

  public Elevator() {
    ConfigMotor(motor);
    motor.setSensorPhase(false);
    motor.setInverted(true);
    motor.setSelectedSensorPosition(0);


    motorFollower.set(ControlMode.Follower, motor.getDeviceID());
    motorFollower.setSensorPhase(false);
    motorFollower.setInverted(true);
  }

  private void ConfigMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative,
      RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
    );

    
    /* set allowed voltage */
    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
		motor.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);
    
    /* how wrong motor is allowed to be */
    motor.configAllowableClosedloopError(0, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);

    /* set variables for PID loops */
    motor.config_kF(RobotMap.PIDLoopIdx, RobotMap.PIDkF, RobotMap.TimeoutMs);
		motor.config_kP(RobotMap.PIDLoopIdx, RobotMap.PIDkP, RobotMap.TimeoutMs);
		motor.config_kI(RobotMap.PIDLoopIdx, RobotMap.PIDkI, RobotMap.TimeoutMs);
    motor.config_kD(RobotMap.PIDLoopIdx, RobotMap.PIDkD, RobotMap.TimeoutMs);

    motor.configMotionAcceleration(3000, RobotMap.TimeoutMs);
    motor.configMotionCruiseVelocity(5000, RobotMap.TimeoutMs);
  }

  /**
   * Sets tele op command to driveTeleOpCommand
   */
  @Override public void initDefaultCommand() {
    setDefaultCommand(new ElevatorTeleOpCommand(ElevatorPosition.DOWN));
  }

  public void teleop() {

    if(Math.abs(OI.operatorStick.getRawAxis(5))<0.1){
      motor.set(ControlMode.Velocity, 0);
      SmartDashboard.putNumber("Encoder", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
    }
    else{
      if(OI.operatorStick.getRawAxis(5)<-0.1){
        if(motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx)<0){
          double percentOutput = -OI.operatorStick.getRawAxis(5);
          SmartDashboard.putNumber("percentOutput", percentOutput);
          motor.set(ControlMode.PercentOutput, percentOutput);
          SmartDashboard.putNumber("Encoder", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
        }
      }
      else if(OI.operatorStick.getRawAxis(5)>0.1){
        if(motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx)>RobotMap.elevatorMaxPosition){
          double percentOutput = -OI.operatorStick.getRawAxis(5);
          SmartDashboard.putNumber("percentOutput", percentOutput);
          motor.set(ControlMode.PercentOutput, percentOutput);
          SmartDashboard.putNumber("Encoder", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
        }
        else{
          motor.set(ControlMode.Velocity, 0);
        }
      }

    }
  }

  public void goToPositon(ElevatorPosition position) {
    if (position == ElevatorPosition.DOWN) {
      System.out.println("DOWN");
      motor.set(ControlMode.MotionMagic, -500);
      SmartDashboard.putNumber("Encoder", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));

    } else if (position == ElevatorPosition.UP) {
      System.out.println("UP");
      motor.set(ControlMode.MotionMagic, -45000);
      SmartDashboard.putNumber("Encoder", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));

    }
  }

  public double getPosition() {
    //return motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx);
    return 0;
  }

  public boolean isPositon(ElevatorPosition position) {
    if (position == ElevatorPosition.DOWN) {
      return getPosition() == 0;
    } else if (position == ElevatorPosition.UP) {
      return getPosition() == RobotMap.elevatorMaxPosition;
    } else {
      return false;
    }
  }

  public void dump() {
    //SmartDashboard.putNumber("current elevator position", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
    //SmartDashboard.putNumber("current elevator velocity", motor.getSelectedSensorVelocity(RobotMap.PIDLoopIdx));
  }
}