package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmFlipCommand;
import frc.robot.commands.ClampGrabCommand;
import frc.robot.commands.LiftFlipCommand;
import frc.robot.commands.DriverFlipDirectionCommand;
import frc.robot.commands.ElevatorTeleOpCommand;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	
	public static final Joystick driverStick = new Joystick(0);
	public static final Joystick operatorStick = new Joystick(1);

	public static final JoystickButton liftButton = new JoystickButton(operatorStick, 1);

	public static final JoystickButton toggleDriverDirection = new JoystickButton(driverStick, 1);

	public static final JoystickButton openClampButton = new JoystickButton(operatorStick, 6);
	public static final JoystickButton closeClampButton = new JoystickButton(operatorStick, 5);

	public static final JoystickButton upPosition = new JoystickButton(operatorStick, 2);
	public static final JoystickButton downPosition = new JoystickButton(operatorStick, 3);

	
	
	public OI() {
		liftButton.whenPressed(new LiftFlipCommand());

		upPosition.whenPressed(new ElevatorTeleOpCommand(Elevator.ElevatorPosition.UP));
		downPosition.whenPressed(new ElevatorTeleOpCommand(Elevator.ElevatorPosition.DOWN));


		toggleDriverDirection.whenPressed(new DriverFlipDirectionCommand());


		ClampGrabCommand clampGrab = new ClampGrabCommand(ClampGrabCommand.GrabStatus.GRAB);
		openClampButton.whileHeld(clampGrab);
//		openClampButton.whenR
//		Pressed(clampGrab);

		ClampGrabCommand clampRelease = new ClampGrabCommand(ClampGrabCommand.GrabStatus.RELEASE);
		closeClampButton.whileHeld(clampRelease);
//		openClampButton.cancelWhenPressed(clampRelease);

	}
}