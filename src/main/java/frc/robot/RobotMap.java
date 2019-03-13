/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Variables for the PWM Ports
  public static class PwmPorts {
    public static int leftMotor1 = 0;
    public static int leftMotor2 = 1;
    public static int rightMotor1 = 2;
    public static int rightMotor2 = 3;
    public static int liftMotor1 = 4;
    public static int liftMotor2 = 5;
    // ...
    public static int servo = 6;
    // ...
    public static int leftIntake = 8;
    public static int rightIntake = 9;
  }

  // Variables for the DIO Ports
  public static class DioPorts {
    // ...
    public static int leftEncoderAChannel = 2;
    public static int leftEncoderBChannel = 3;

    public static int rightEncoderAChannel = 4;
    public static int rightEncoderBChannel = 5;

    public static int liftEncoderAChannel = 6;
    public static int liftEncoderBChannel = 7;
    // ...
    public static int liftHigh = 8;
    public static int liftLow = 9; 
  }

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
