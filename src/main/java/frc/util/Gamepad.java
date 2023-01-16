/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.wpilibj.Joystick;

public class Gamepad extends Joystick {

  public Gamepad(int port) {
    super(port);
  }

  // GAMEPAD BUTTONS
  public static final int kGamepadButtonX = 1;
  public static final int kGamepadButtonA = 2; 
  public static final int kGamepadButtonB = 3; 
  public static final int kGamepadButtonY = 4; 
  public static final int kGamepadButtonShoulderL = 5;
  public static final int kGamepadButtonShoulderR = 6; 
  public static final int kGamepadTriggerLeft = 7;
  public static final int kGamepadTriggerRight = 8; 
  public static final int kGamepadButtonBack = 9;
  public static final int kGamepadButtonStart = 10; 
  public static final int kGamepadLeftStick = 11;
  public static final int kGamepadRightStick = 12;
  

  public double getRawAxis(int axis) {
    return super.getRawAxis(axis);
  }

  public double getLeftX() {
    return super.getRawAxis(0);
  }

  public double getLeftY() {
    return super.getRawAxis(1);
  }

  public double getRightX() {
    return super.getRawAxis(2);
  }

  public double getRightY() {
    return super.getRawAxis(3);
  }

}