package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

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

  public double getPIDControlError(double distance) {
    double distanceDriven =
      (
        this.rightMotor.getSelectedSensorPosition() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION +
        this.leftMotor.getSelectedSensorPosition() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION
      ) /
      2;
    return distance - distanceDriven;
  }

  public void driveDistance(double distance) {
    double distanceDriven =
      (
        this.rightMotor.getSelectedSensorPosition() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION +
        this.leftMotor.getSelectedSensorPosition() *
        RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION
      ) /
      2;
    double error = distance - distanceDriven;
    double velocity = 0.5 * error;

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Velocity", velocity);

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

  public void getSmartDashboardValues() {
    SmartDashboard.putNumber(
      "Distance Traveled",
      this.rightMotor.getSelectedSensorPosition() *
      RobotMap.DRIVETRAIN.TICK_TO_FEET_CONVERSION
    );
    SmartDashboard.putNumber(
      "Velocity",
      (
        this.rightMotor.getSelectedSensorVelocity() /
        RobotMap.DRIVETRAIN.VELOCITY_CONVERSION +
        this.leftMotor.getSelectedSensorVelocity() /
        RobotMap.DRIVETRAIN.VELOCITY_CONVERSION
      ) /
      2
    );
    SmartDashboard.putNumber(
      "Right Enc Vel.",
      this.rightMotor.getSelectedSensorVelocity() /
      RobotMap.DRIVETRAIN.VELOCITY_CONVERSION
    );
    SmartDashboard.putNumber(
      "Left Enc Vel.",
      this.leftMotor.getSelectedSensorVelocity() /
      RobotMap.DRIVETRAIN.VELOCITY_CONVERSION
    );
  }
}
