package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArmTeleOpCommand;

public class Vacuum extends Subsystem {
  public static final TalonSRX motor = new TalonSRX(RobotMap.clamp);
  public static final Solenoid valve = new Solenoid(0);

  @Override public void initDefaultCommand() {
    class TeleOp extends Command {
        public TeleOp() {
            requires(Robot.vacuum);
        }

        @Override
        protected void execute() {
            // grab
            if (OI.operatorStick.getRawButtonPressed(1)) {
                motor.set(ControlMode.PercentOutput, 1);
                // valve.set(false);
            }

            // valve close
            if (OI.operatorStick.getRawButtonPressed(3)) {
                valve.set(false);
            }

            // release
            if (OI.operatorStick.getRawButtonPressed(2)) {
                motor.set(ControlMode.PercentOutput, 0);
                // valve.set(true);
            }

            // valve open
            if (OI.operatorStick.getRawButtonPressed(4)) {
                valve.set(true);
            }
        }


        @Override
        protected boolean isFinished() {
            return false;
        }
    }

    setDefaultCommand(new TeleOp());
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
    valve.set(false);
  }
}