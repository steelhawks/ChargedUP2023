package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

  private double errorSum = 0;
  private double error = 0;

  private final WPI_TalonSRX rightMotor;
  private final WPI_TalonSRX leftMotor;

  private final DifferentialDrive drive;

  public Drivetrain() {
    this.rightMotor =
      new WPI_TalonSRX(RobotMap.DRIVETRAIN.MOTOR_RIGHT_ONE_PORT);
    this.leftMotor = new WPI_TalonSRX(RobotMap.DRIVETRAIN.MOTOR_LEFT_ONE_PORT);

    this.rightMotor.setSelectedSensorPosition(0);
    this.leftMotor.setSelectedSensorPosition(0);

    configureMotors();
    zeroEncoders();

    this.drive = new DifferentialDrive(this.leftMotor, this.rightMotor);
  }

  public void arcadeDrive(Joystick stick) {
    this.drive.arcadeDrive(-stick.getY() * 0.63, -stick.getTwist() / 2);
  }

  public void zeroEncoders() {
    this.rightMotor.setSelectedSensorPosition(0, 0, 10);
    this.leftMotor.setSelectedSensorPosition(0, 0, 10);
  }

  public void driveDistance(double distance) {
    double integralLimit = 1;
    double distanceDriven =
      (
        this.rightMotor.getSelectedSensorPosition() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION +
        this.leftMotor.getSelectedSensorPosition() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION
      ) /
      2;
    double error = distance - distanceDriven;
    this.error = error;

    if (integralLimit > error) this.errorSum = this.errorSum + error;

    double velocity = 0.5 * error + 0.1 * this.errorSum;

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Error Sum", this.errorSum);
    SmartDashboard.putNumber("Distance Driven", distanceDriven);

    this.rightMotor.set(velocity);
    this.leftMotor.set(velocity);
  }

  public void manualDrive(double rightMotorSpeed, double leftMotorSpeed) {
    this.rightMotor.set(rightMotorSpeed);
    this.leftMotor.set(leftMotorSpeed);
  }

  public void configureMotors() {
    this.rightMotor.configFactoryDefault();
    this.leftMotor.configFactoryDefault();

    this.leftMotor.setNeutralMode(NeutralMode.Brake);
    this.rightMotor.setNeutralMode(NeutralMode.Brake);

    this.rightMotor.setInverted(true);
  }

  public void getDashboard() {
    SmartDashboard.putNumber(
      "Velocity",
      (
        this.rightMotor.getSelectedSensorVelocity() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION +
        this.leftMotor.getSelectedSensorVelocity() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION
      ) /
      2
    );
  }

  public double getPIDControlError() {
    return this.error;
  }

  public void resetErrorSum() {
    this.errorSum = 0;
  }
}
