/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class DriveSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DifferentialDrive m_diffDrive;
  AHRS m_ahrs;
  public PIDController m_anglePidController;


  Encoder m_driveEncLeft = new Encoder(RobotMap.DioPorts.leftEncoderAChannel, RobotMap.DioPorts.leftEncoderBChannel,
      true, Encoder.EncodingType.k2X);
  Encoder m_driveEncRight = new Encoder(RobotMap.DioPorts.rightEncoderAChannel, RobotMap.DioPorts.rightEncoderBChannel,
      true, Encoder.EncodingType.k2X);    

  //DigitalInput m_encoderTester2 = new DigitalInput(2);
  //DigitalInput m_encoderTester4 = new DigitalInput(4);
  //DigitalInput m_encoderTester3 = new DigitalInput(3);
 // DigitalInput m_encoderTester5 = new DigitalInput(5);
    public DriveSystem() {

    SpeedController left1 = new VictorSP(RobotMap.PwmPorts.leftMotor1);
    SpeedController left2 = new VictorSP(RobotMap.PwmPorts.leftMotor2);
    SpeedController right1 = new VictorSP(RobotMap.PwmPorts.rightMotor1);
    SpeedController right2 = new VictorSP(RobotMap.PwmPorts.rightMotor2);

    SpeedControllerGroup leftGroup = new SpeedControllerGroup(left1, left2);
    SpeedControllerGroup rightGroup = new SpeedControllerGroup(right1, right2);

    m_diffDrive = new DifferentialDrive(leftGroup, rightGroup);

    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       ************************************************************************/
      m_ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 50);
      m_ahrs.enableLogging(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    m_anglePidController = new PIDController(0.35, 0.0, 0.0, m_ahrs, d -> this.usePIDAngleOutput(d));

    m_anglePidController.setAbsoluteTolerance(0.3);
    m_anglePidController.setInputRange(-180, +180);
    m_anglePidController.setContinuous(true);
    m_anglePidController.setOutputRange(-1, 1);

    // LiveWindow.addSensor("DriveSystem", "IMU", m_ahrs);
    // LiveWindow.addActuator("DriveSystem", "AnglePID", m_anglePidController);

    double dEncDistancePerPulse = 19.25 / 360;
    m_driveEncLeft.setDistancePerPulse(dEncDistancePerPulse);
    m_driveEncRight.setDistancePerPulse(dEncDistancePerPulse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new TankDriveCommand());
    //setDefaultCommand(new ArcadeDriveCommand());
    //setDefaultCommand(new CurvatureDriveCommand());
    setDefaultCommand(new ChooserDriveCommand());
  }

  public void tankDrive(Joystick leftStick, Joystick rightStick) {
    this.m_anglePidController.disable();

    double throttle = leftStick.getRawAxis(OI.JoystickAxis.Throttle) / 4 + 0.75;
    SmartDashboard.putNumber("Throttle", throttle);

    double leftSpeed = leftStick.getRawAxis(OI.JoystickAxis.UpDown) * throttle;
    double rightSpeed = rightStick.getRawAxis(OI.JoystickAxis.UpDown) * throttle;

    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(Joystick joystick) {
    this.m_anglePidController.disable();

    double Speed = joystick.getRawAxis(OI.JoystickAxis.UpDown);
    double Rotation = joystick.getRawAxis(OI.JoystickAxis.LeftRight);

    m_diffDrive.arcadeDrive(Speed, Rotation);
  }

  public void curvatureDrive(Joystick joystick) {
    this.m_anglePidController.disable();

    // double throttle = joystick.getRawAxis(OI.JoystickAxis.Throttle)/4 + 0.75;
    // SmartDashboard.putNumber("Throttle", throttle);
    // double speed = joystick.getRawAxis(OI.JoystickAxis.UpDown) * throttle;
    // double Rotation = joystick.getRawAxis(OI.JoystickAxis.LeftRight) *throttle;

    double speed = joystick.getRawAxis(OI.ArcadeStickAxis.UpDown);
    double Rotation = joystick.getRawAxis(OI.ArcadeStickAxis.turn);

    boolean x = joystick.getRawButton(1);

    m_diffDrive.curvatureDrive(speed, Rotation, x);
  }

  public void arcadeStickArcadeDrive(Joystick joystick){
    this.m_anglePidController.disable();

    double Speed = joystick.getRawAxis(OI.ArcadeStickAxis.UpDown);
    double Rotation = joystick.getRawAxis(OI.ArcadeStickAxis.turn);

    m_diffDrive.arcadeDrive(Speed, Rotation);
  }

  public void noTurnCurvatureDrive(Joystick joystick){
    this.m_anglePidController.disable();

    double Speed = joystick.getRawAxis(OI.JoystickAxis.UpDown);
    double Rotation = joystick.getRawAxis(OI.JoystickAxis.LeftRight);

    boolean x = joystick.getRawButton(1);

    m_diffDrive.curvatureDrive(Speed, Rotation, x);
  }

  public void log() {
    // SmartDashboard.putNumber("left encoder", this.m_driveEncLeft.get());
    // SmartDashboard.putNumber("right encoder", this.m_driveEncRight.get());
    
  //  SmartDashboard.putBoolean("Tester2", this.m_encoderTester2.get());
    // SmartDashboard.putBoolean("Tester4", this.m_encoderTester4.get());
    // SmartDashboard.putBoolean("Tester3", this.m_encoderTester3.get());
    // SmartDashboard.putBoolean("Tester5", this.m_encoderTester5.get());

    /* Display 6-axis Processed Angle Data                                      */
    // SmartDashboard.putBoolean(  "IMU_Connected",        m_ahrs.isConnected());
    // SmartDashboard.putBoolean(  "IMU_IsCalibrating",    m_ahrs.isCalibrating());
    // SmartDashboard.putNumber(   "IMU_Yaw",              m_ahrs.getYaw());
    // SmartDashboard.putNumber(   "IMU_Pitch",            m_ahrs.getPitch());
    // SmartDashboard.putNumber(   "IMU_Roll",             m_ahrs.getRoll());
    
    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
    
    SmartDashboard.putNumber(   "IMU_TotalYaw",         m_ahrs.getAngle());
    //SmartDashboard.putNumber(   "IMU_YawRateDPS",       m_ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
    // SmartDashboard.putNumber(   "IMU_Accel_X",          m_ahrs.getWorldLinearAccelX());
    // SmartDashboard.putNumber(   "IMU_Accel_Y",          m_ahrs.getWorldLinearAccelY());
    // SmartDashboard.putBoolean(  "IMU_IsMoving",         m_ahrs.isMoving());
    // SmartDashboard.putBoolean(  "IMU_IsRotating",       m_ahrs.isRotating());
    
    //SmartDashboard.putNumber(   "IMU_Temp_C",           m_ahrs.getTempC());
    
    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    //AHRS.BoardYawAxis yaw_axis = m_ahrs.getBoardYawAxis();
    //SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    //SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
    
    // /* Sensor Board Information                                                 */
    //SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
    
    // /* Quaternion Data                                                          */
    // /* Quaternions are fascinating, and are the most compact representation of  */
    // /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    // /* from the Quaternions.  If interested in motion processing, knowledge of  */
    // /* Quaternions is highly recommended.                                       */
    // SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
    // SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
    // SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
    // SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
    
    // /* Connectivity Debugging Support                                           */
    // SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
    // SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
    
  }

  // public void turnToAngle(double targetAngle){
  //   double currentAngle = this.m_ahrs.getAngle();
  //   double c2 = currentAngle;
  //   if(Math.abs(targetAngle-currentAngle) > 180){
  //     if(currentAngle > 0){
  //       c2 = currentAngle - 360;
  //     }
  //     else {
  //       c2 = 360 + currentAngle;
  //     }
  //   }

  // }

  public void stop() {
    m_diffDrive.tankDrive(0, 0);
  }

  // @Override
  // protected double returnPIDInput() {
  //   return m_ahrs.getAngle();
  // }

  //@Override
  protected void usePIDAngleOutput(double output) {
    m_diffDrive.arcadeDrive(0, output / 2);
   // LiveWindow.addChild(this, output);
   // LiveWindow.addChild(this, ahrs.getAngle());
  }

}
