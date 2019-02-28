/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeSystem extends Subsystem {
  // Constants
  private static double intakeSpeed = 1;
  
  // Components of the subsystem
  private SpeedController leftMotor = new VictorSP(RobotMap.PwmPorts.leftIntake);
  private SpeedController rightMotor = new VictorSP(RobotMap.PwmPorts.rightIntake);

  // Put methods for controlling this subsystem here. Call these from Commands.
  public void intake() {
    this.rightMotor.set(intakeSpeed);
    this.leftMotor.set(-1*intakeSpeed);
  }

  public void extake() {
    this.rightMotor.set(-1*intakeSpeed);
    this.leftMotor.set(intakeSpeed);
  }

  public void stop() {
    this.rightMotor.set(0);
    this.leftMotor.set(0);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
