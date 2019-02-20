package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmFlipCommand;
import frc.robot.commands.ClampGrabCommand;
import frc.robot.commands.LiftUnlockCommand;
import frc.robot.commands.DriverFlipDirectionCommand;
import frc.robot.commands.ElevatorTeleOpCommand;
import frc.robot.commands.LiftJumpCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	
	public static final Joystick driverStick = new Joystick(0);
	public static final Joystick operatorStick = new Joystick(1);

	public static final JoystickButton liftUnlockButton = new JoystickButton(operatorStick, 14);
	public static final JoystickButton liftJumpButton = new JoystickButton(operatorStick, 4);

	public static final JoystickButton toggleDriverDirection = new JoystickButton(driverStick, 1);

	public static final JoystickButton releaseClampButton = new JoystickButton(operatorStick, 3);
	public static final JoystickButton grabClampButton = new JoystickButton(operatorStick, 2);

	public static final POVButton upPosition = new POVButton(operatorStick, 0);
	public static final POVButton downPosition = new POVButton(operatorStick, 180);
	public static final POVButton midPosition = new POVButton(operatorStick, 90);

	public static final JoystickButton backArm = new JoystickButton(operatorStick, 5);
	public static final POVButton centerArm = new POVButton(operatorStick, 270);
	public static final JoystickButton frontArm = new JoystickButton(operatorStick, 6);

	
	
	public OI() {
		gyro.reset();
		gyro.calibrate(); 

		liftUnlockButton.whenPressed(new LiftUnlockCommand());
		liftJumpButton.whileHeld(new LiftJumpCommand());

		upPosition.whenPressed(new ElevatorTeleOpCommand(Elevator.ElevatorPosition.UP));
		downPosition.whenPressed(new ElevatorTeleOpCommand(Elevator.ElevatorPosition.DOWN));
		midPosition.whenPressed(new ElevatorTeleOpCommand(Elevator.ElevatorPosition.MID));

		backArm.whenPressed(new ArmFlipCommand(Arm.ArmPosition.BACK));
		centerArm.whenPressed(new ArmFlipCommand(Arm.ArmPosition.CENTER));
		frontArm.whenPressed(new ArmFlipCommand(Arm.ArmPosition.FRONT));

		toggleDriverDirection.whenPressed(new DriverFlipDirectionCommand());

		ClampGrabCommand clampRelease = new ClampGrabCommand(ClampGrabCommand.GrabStatus.RELEASE);
		releaseClampButton.whenPressed(clampRelease);
		ClampGrabCommand clampGrab = new ClampGrabCommand(ClampGrabCommand.GrabStatus.GRAB);
		grabClampButton.whenPressed(clampGrab);
	}

	public static void dump() {
		//gyro.hashCode()
		SmartDashboard.putNumber("angle", gyro.getRate());
	}
}