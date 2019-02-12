package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.LiftFlipCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	
	public static final Joystick driverStick = new Joystick(0);
	public static final Joystick operatorStick = new Joystick(1);

	public static final JoystickButton liftButton = new JoystickButton(driverStick, 1);
	
	public OI() {
		liftButton.whenPressed(new LiftFlipCommand());
	}
}