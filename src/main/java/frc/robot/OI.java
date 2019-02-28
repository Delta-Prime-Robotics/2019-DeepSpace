/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // *** Opertor Cameras ***
  public static UsbCamera camera1;
  public static UsbCamera camera2;

  // *** Controller constants ***
  private static class UsbPort {
    public static int leftStick = 1;
    public static int rightStick = 0;
    public static int gamePad = 2;
    public static int arcadeStick = 3;
  }
  
  public static class GamePadAxis {
    public static int LeftStickLR = 0;
    public static int LeftStickUpDn = 1;
    public static int RightStickLR = 2;
    public static int RightStickUpDn = 3;
  }
  private static class GamePadButton {
    public static int X = 1;
    public static int A = 2;
    public static int B = 3;
    public static int Y = 4;
    // public static int LB = 5;
    // public static int RB = 6;
    // public static int LT = 7;
    // public static int RT = 8;
    // public static int Back = 9;
    // public static int Start = 10;
    // public static int LeftJoyStickClick = 11;
    // public static int RightJoyStickClick = 12;
  }

  // private static class ArcadeStickButtons {
  //   public static int Thumb = 2;
  // }
  public static class JoystickAxis {
    public static int LeftRight = 0;
    public static int UpDown = 1;
    public static int Throttle = 2;
  }

  public static class ArcadeStickAxis{
    public static int UpDown = 1;
    public static int turn = 2;
    //public static int arcadeThrottle = 3;
  }
  // The numbers on the Joystick buttons match the Ids that we'd use
  // So we don't need to define those as constants (One = 1, etc. ;P)


  // *** Controllers ***
  public Joystick leftStick = new Joystick(UsbPort.leftStick);
  public Joystick rightStick = new Joystick(UsbPort.rightStick);
  public Joystick gamePad = new Joystick(UsbPort.gamePad);
  public Joystick arcadeStick = new Joystick(UsbPort.arcadeStick);

  // *** Mapping Buttons to Commands ***
  JoystickButton intakeButton = new JoystickButton(gamePad, GamePadButton.Y);
  JoystickButton extakeButton = new JoystickButton(gamePad, GamePadButton.A);
  
  JoystickButton openServo = new JoystickButton(gamePad, GamePadButton.B);
  JoystickButton closeServo = new JoystickButton(gamePad, GamePadButton.X);
  
  JoystickButton quickTurn = new JoystickButton(arcadeStick, 2);
   // JoystickButton arcadeOn = new JoystickButton(arcadeStick, ArcadeStickButtons.Thumb);
  

  public OI() {
    intakeButton.whileHeld(new IntakeInCommand());
    extakeButton.whileHeld(new IntakeOutCommand());
    
    openServo.whenPressed(new SetServoAngleCommand(180));
    closeServo.whenPressed(new SetServoAngleCommand(0));

    //arcadeOn.whileHeld(new ArcadeDriveCommand());
    // Cameras
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);

    SmartDashboard.putData("Turn To 35", new TurnToAngleCommand());
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
