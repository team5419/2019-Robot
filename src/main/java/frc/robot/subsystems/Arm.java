package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.ArmFlipCommand;

public class Arm extends Subsystem {
  TalonSRX motor = new TalonSRX(RobotMap.arm);

  int target = 0;
  ArmPosition status = ArmPosition.BACK;

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
      if(Clamp.isGrab){
        target += OI.operatorStick.getRawAxis(1) * 10;
        if (target < 0) {
          target = 0;
        }
        if (target > 1900) {
          target = 1900;
        }
        motor.set(ControlMode.MotionMagic, target);
      }


    // if(Clamp.isGrab){
    //   if(Math.abs(OI.operatorStick.getRawAxis(1))<0.1){
    //     motor.set(ControlMode.PercentOutput, -0.1);
    //   }
    //   else{
    //     double percentOutput = -OI.operatorStick.getRawAxis(1) / 2;
    //     motor.set(ControlMode.PercentOutput, percentOutput);
    //   }
    //   //motor.set(Mode, demand);
    // }
  }

  private void ConfigMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative,
      RobotMap.PIDLoopIdx, RobotMap.TimeoutMs
    );

    // Reset Encoder
    motor.setSelectedSensorPosition(0);
    
    /* how wrong motor is allowed to be */
    motor.configAllowableClosedloopError(0, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);

    /* set variables for PID loops */
    motor.config_kF(RobotMap.PIDLoopIdx, RobotMap.PIDkF, RobotMap.TimeoutMs);
		motor.config_kP(RobotMap.PIDLoopIdx, RobotMap.PIDkP, RobotMap.TimeoutMs);
		motor.config_kI(RobotMap.PIDLoopIdx, RobotMap.PIDkI, RobotMap.TimeoutMs);
    motor.config_kD(RobotMap.PIDLoopIdx, RobotMap.PIDkD, RobotMap.TimeoutMs);

    motor.configMotionAcceleration(1000, RobotMap.TimeoutMs);
    motor.configMotionCruiseVelocity(3000, RobotMap.TimeoutMs);
  }

  @Override public void initDefaultCommand() {
    setDefaultCommand(new ArmFlipCommand());
  }

  public void flip(ArmPosition position) {
    if(Clamp.isGrab){
      if (position == ArmPosition.BACK) {
        System.out.println("BACK");
        target = 0;
        motor.set(ControlMode.MotionMagic, 0);
      } else if (position == ArmPosition.CENTER) {
        System.out.println("CENTER");
        int up_pos = 950;
        target = up_pos;
        motor.set(ControlMode.MotionMagic, up_pos);
      } else if (position == ArmPosition.FRONT){
        System.out.println("FRONT");
        int up_pos = 1900;
        target = up_pos;
        motor.set(ControlMode.MotionMagic, up_pos); 
      }
    }
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void dump() {
    SmartDashboard.putNumber("Arm Encoder", motor.getSelectedSensorPosition(RobotMap.PIDLoopIdx));
  }
}
