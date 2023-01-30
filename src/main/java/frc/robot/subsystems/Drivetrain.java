package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private final WPI_TalonSRX rightMotorOne;
    private final WPI_TalonSRX leftMotorOne;
    private final WPI_TalonSRX rightMotorTwo;
    private final WPI_TalonSRX leftMotorTwo;

    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;

    private final DifferentialDrive drive;

    public Drivetrain() {
        this.rightMotorOne = new WPI_TalonSRX(Constants.Motors.MOTOR_RIGHT_ONE_PORT);
        this.leftMotorOne = new WPI_TalonSRX(Constants.Motors.MOTOR_LEFT_ONE_PORT);
        this.rightMotorTwo = new WPI_TalonSRX(Constants.Motors.MOTOR_RIGHT_TWO_PORT);
        this.leftMotorTwo = new WPI_TalonSRX(Constants.Motors.MOTOR_LEFT_TWO_PORT);

        this.leftMotors = new MotorControllerGroup(leftMotorOne, leftMotorTwo);
        this.rightMotors = new MotorControllerGroup(rightMotorOne, rightMotorTwo);

        this.drive = new DifferentialDrive(leftMotors, rightMotors);
    }

    public void arcadeDrive(Joystick stick) {
        this.drive.arcadeDrive(stick.getY(), -stick.getTwist());
    }
}
