package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.ArmFlipCommand;

public class Arm extends Subsystem {
  TalonSRX motor = new TalonSRX(RobotMap.arm);

  int target = 0;
  ArmPosition status = ArmPosition.CENTER;

  public enum ArmPosition {
    FRONT,
    CENTER,
    BACK
  }

  public Arm() {
    motor.configNominalOutputForward(0, RobotMap.TimeoutMs);
		motor.configNominalOutputReverse(0, RobotMap.TimeoutMs);
		motor.configPeakOutputForward(RobotMap.percent, RobotMap.TimeoutMs);
    motor.configPeakOutputReverse(-RobotMap.percent, RobotMap.TimeoutMs);

    motor.setInverted(true);
    
    ConfigMotor(motor);
  }

  public void teleOp() {
    //if(Clamp.isGrab){
      if(Math.abs(OI.operatorStick.getRawAxis(1))<0.1){
        motor.set(ControlMode.PercentOutput, -0.1);
      }
      else{
        double percentOutput = -OI.operatorStick.getRawAxis(1) / 2;
        motor.set(ControlMode.PercentOutput, percentOutput);
      }
      //motor.set(Mode, demand);
    //}
  }

  private void ConfigMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative,
      RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
    );
    
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

  @Override public void initDefaultCommand() {
    setDefaultCommand(new ArmFlipCommand());
  }

  //public void flipTowardFront() {
  //   motor.set(ControlMode.Position, 0);
  // }

  // public void flipTowardBack(){
  //   motor.set(ControlMode.Position, 100);
  // }

  public void flip(ArmPosition position) {
    if (position == ArmPosition.BACK) {
      target = 0;
      status = ArmPosition.BACK;
    } else if (position == ArmPosition.FRONT) {
      target = 0;
      status = ArmPosition.BACK;
    }
    motor.set(ControlMode.Position, target);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.2);
  }
}
