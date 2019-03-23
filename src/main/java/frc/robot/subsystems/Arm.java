package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArmFlipCommand;

public class Arm extends Subsystem {
  public static final TalonSRX motor = new TalonSRX(RobotMap.arm);

  public int target = -1;
  int max = 2000;
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
    if (target == -1) {
      target = motor.getSelectedSensorPosition();
    }
    double d = OI.operatorStick.getRawAxis(1);
    if (Math.abs(d) < .25) {
      d = 0;
    }
    target += d;
    if (target < 0) {
      target = 0;
    }
    if (target > max) {
      target = max;
    }
    if (Robot.elevator.target == -500 && Elevator.motor.getSelectedSensorPosition() < -600) {
      target = max;
    }
    motor.set(ControlMode.MotionMagic, target);
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
		motor.config_kP(RobotMap.PIDLoopIdx, 1.5, RobotMap.TimeoutMs);
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
        int middle_pos = 950;
        target = middle_pos;
        motor.set(ControlMode.MotionMagic, middle_pos);
      } else if (position == ArmPosition.FRONT){
        System.out.println("FRONT");
        int up_pos = max;
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
    SmartDashboard.putNumber("Arm current", motor.getOutputCurrent());
  }
}
