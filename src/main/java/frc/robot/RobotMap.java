/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap 
{
  /*****
   * Drivetrain instance variables
   *****/

  //Right Motor Ports
  private final int RIGHT_MOTOR_PORT_ONE = 0;

  //Left Motor Ports
  private final int LEFT_MOTOR_PORT_ONE = 1;

  //Optical Encoder Ports
  private final int LEFT_ENC_PORT_A = 0;
  private final int LEFT_ENC_PORT_B = 1; 
  private final int RIGHT_ENC_PORT_A = 2;
  private final int RIGHT_ENC_PORT_B = 3;

  //Gyro
  private final double KP_GYRO = 0.008;
  

  /*****
   * Ultrasonic instance variables
   *****/

  private final int ULTRA_PING = 8;
  private final int ULTRA_ECHO = 9;

  /*****
   * Driverstation instance variables
   *****/

  //Input Ports

  private final int JOYSTICK_PORT_ONE = 0;

  //Button ports
  private final int SHIFT_BUTTON = 1;
  private final int ALIGN_BUTTON = 3;
  private final int BAY_BUTTON = 7;
  private final int PORT_BUTTON = 11;
  private final int BALL_BUTTON = 9;


  /*****
   * Constructor methods
   *****/
  public RobotMap() {}

  /*****
   * Getter methods
   *****/


  public int getRightMotorPortOne()
  {
    return this.RIGHT_MOTOR_PORT_ONE;
  }

  public int getLeftMotorPortOne()
  {
    return this.LEFT_MOTOR_PORT_ONE;
  }

  public int getLeftEncPortA()
  {
    return this.LEFT_ENC_PORT_A;
  }
  public int getLeftEncPortB()
  {
    return this.LEFT_ENC_PORT_B;
  }

  public int getRightEncPortA()
  {
    return this.RIGHT_ENC_PORT_A;
  }

  public int getRightEncPortB()
  {
    return this.RIGHT_ENC_PORT_B;
  }

  public double getKPGyro()
  {
    return this.KP_GYRO;
  }

  public int getUltraPing()
  {
    return this.ULTRA_PING;
  }

  public int getUltraEcho()
  {
    return this.ULTRA_ECHO;
  }

  public int getJoystickPortOne()
  {
    return this.JOYSTICK_PORT_ONE;
  }

  public int getShiftButton()
  {
    return this.SHIFT_BUTTON;
  }

  public int getAlignButton()
  {
    return this.ALIGN_BUTTON;
  }

  public int getBallButton() 
  {
    return this.BALL_BUTTON;
  }

  public int getPortButton() 
  {
    return this.PORT_BUTTON;
  } 

  public int getBayButton() 
  {
    return this.BAY_BUTTON;
  }

}
