package frc.robot;

public class RobotMap {

  public final int drivetrainLeftMotorPortOne = 1;
  public final int drivetrainLeftMotorPortTwo = 2;
  public final int drivetrainLeftMotorPortThree = 3;
  
  public final int drivetrainRightMotorPortOne = 4;
  public final int drivetrainRightMotorPortTwo = 5;
  public final int drivetrainRightMotorPortThree = 6;

  public final int drivetrainSolenoidPortOn = 0; //make sure it is right
  public final int drivetrainSolenoidPortOff = 1;

  private final double KPGyro = 0.008;

  private final int joystickOnePort = 0;

  //Button Ports
  private final int shiftButton = 1;

  public RobotMap() {}

  public int getdrivetrainLeftMotorPortOne() {
    return this.drivetrainLeftMotorPortOne;
  }

  public int getdrivetrainLeftMotorPortTwo() {
    return this.drivetrainLeftMotorPortTwo;
  }

  public int getdrivetrainLeftMotorPortThree() {
    return this.drivetrainLeftMotorPortThree;
  }

  public int getdrivetrainRightMotorPortOne() {
    return this.drivetrainRightMotorPortOne;
  }

  public int getdrivetrainRightMotorPortTwo() {
    return this.drivetrainRightMotorPortTwo;
  }

  public int getdrivetrainRightMotorPortThree() {
    return this.drivetrainRightMotorPortThree;
  }

  public double getKPGyro() {
    return this.KPGyro;
  }

  public int getjoystickOnePort() {
    return this.joystickOnePort;
  }

  public int getShiftButton() {
    return this.shiftButton;
  }

}
