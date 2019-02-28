/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ChooserDriveCommand extends Command {
  public ChooserDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveSystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int option = Robot.getDriveOption();
    if(option == 1){
      Robot.m_driveSystem.curvatureDrive(Robot.m_oi.arcadeStick);
    }
    else if(option == 2){
      Robot.m_driveSystem.arcadeDrive(Robot.m_oi.rightStick);
    }
    else if(option == 3){
      Robot.m_driveSystem.tankDrive(Robot.m_oi.leftStick, Robot.m_oi.rightStick);
    }
    else if(option == 4){
      Robot.m_driveSystem.noTurnCurvatureDrive(Robot.m_oi.rightStick);
      //look at it in drive system because the axises are different and right now it may be turning with throttle.
      //make new action in drive for this curvitureDriveRightStick
    }
    else if(option == 5){
      Robot.m_driveSystem.arcadeStickArcadeDrive(Robot.m_oi.arcadeStick);
      //Maybe same problems as last one.
    }


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveSystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
