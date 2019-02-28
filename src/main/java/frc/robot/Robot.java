/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
//import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // OI
  public static OI m_oi;

  // SubSystems
  public static LiftSystem m_liftSystem = new LiftSystem();
  public static IntakeSystem m_intakeSystem = new IntakeSystem();
  public static DriveSystem m_driveSystem = new DriveSystem();
  public static LittleServoSwitch m_littleServoSwitch = new LittleServoSwitch();

  // Autonomous Chooser
  Command m_autonomousCommand;
  // SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Drive Mode Chooser
  private static SendableChooser<Integer> m_driveChooser = new SendableChooser<>(); 


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();

    // Autonomous Chooser
    // m_chooser.setDefaultOption("Default Auto", new DefaultAutoCommand());
    // m_chooser.addOption("My Auto", new MyAutoCommand());
    // SmartDashboard.putData("Auto mode", m_chooser);

    // Drive Mode Chooser
    m_driveChooser.setDefaultOption("Curvature Drive", 1);
    m_driveChooser.addOption("Arcade Drive", 2);
    m_driveChooser.addOption("Tank Drive", 3);
    m_driveChooser.addOption("No Twist Curvature Drive", 4);
    m_driveChooser.addOption("Twist Arcade Drive", 5);
    SmartDashboard.putData("Drive Thingy", m_driveChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want run during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Lift Axis", m_oi.gamePad.getRawAxis(OI.GamePadAxis.RightStickUpDn));
    m_driveSystem.log();
    m_littleServoSwitch.log();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running. 
    // If you want the autonomous to continue until interrupted by another command, 
    // remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_DriveSystem.setDefaultCommand(m_driveChooser.getSelected());
  }

  /**
   * This function is called periodically during operator control.
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

  public static int getDriveOption() {
    return m_driveChooser.getSelected();
  }
}
