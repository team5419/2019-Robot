package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveAutoCommand;
import frc.robot.commands.DriveTurnCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
  private Command autoCommand;
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public static DriveTrain driveTrain;
  public static Elevator elevator;
  public static Lift lift;
  public static Clamp clamp;
  public static Arm arm;

  /**
   * This function is run when the robot is first started up and is
   * used for initialization code.
   */
  @Override
  public void robotInit() {
    // initilize subsystems
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    lift = new Lift();
    clamp = new Clamp();
    arm = new Arm();

    // set up camera server
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);

    // add autonumous modes to chooser
    autoCommandChooser.setDefaultOption("test elevator", new ElevatorAutoCommand());
    autoCommandChooser.addOption("test drive train", new DriveAutoCommand());
    autoCommandChooser.addOption("test turning", new DriveTurnCommand(90));
    SmartDashboard.putData("Auto choices", autoCommandChooser);

    new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    driveTrain.dump();
    elevator.dump();
    OI.dump();
    arm.dump();
    clamp.dump();
    lift.dump();
  }

  /**
   * Called at the start of autonumous mode.
   * It get the selected auto command from autoCommandChooser and runs it
   */
  @Override
  public void autonomousInit() {
    autoCommand = autoCommandChooser.getSelected();
    System.out.print("Running: ");
    System.out.println(autoCommand);
    autoCommand.start();
  }

  /**
   * This function is called periodically during autonomous.
   * Tells driveTrain to update the motion profile buffers
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * Called at the start of teleop mode.
   * It get the selected auto command from autoCommandChooser and runs it
   */
  @Override
  public void teleopInit() {
    if (autoCommand != null) {
			autoCommand.cancel();
		}
  }

  /**
   * This function is called periodically during operator control.
   * It tells the Scheduler to run all the commands choosen in initDefaultCommand by subsystems
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}