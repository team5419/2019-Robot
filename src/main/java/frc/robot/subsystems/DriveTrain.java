/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  public TalonSRX leftBackMotor = new TalonSRX(RobotMap.leftBackMotor);	
  public TalonSRX rightBackMotor = new TalonSRX(RobotMap.rightBackMotor);
  public TalonSRX leftFrontMotor = new TalonSRX(RobotMap.leftFrontMotor);
  public TalonSRX rightFrontMotor = new TalonSRX(RobotMap.rightFrontMotor);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}