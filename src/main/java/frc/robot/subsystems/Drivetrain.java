package frc.robot.subsystems;

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
        this.rightMotor = new WPI_TalonSRX(RobotMap.DRIVETRAIN.MOTOR_RIGHT_ONE_PORT);
        this.leftMotor = new WPI_TalonSRX(RobotMap.DRIVETRAIN.MOTOR_LEFT_ONE_PORT);

        configureMotors();

        this.drive = new DifferentialDrive(this.leftMotor, this.rightMotor);
    }

    public void arcadeDrive(Joystick stick) {
        this.drive.arcadeDrive(stick.getY() * 0.63, -stick.getTwist() / 2);
    }

    public void driveStraight(double speed) {
        this.rightMotor.set(speed);
        this.leftMotor.set(speed);
    }

    public void configureMotors() {
        this.rightMotor.configFactoryDefault();
        this.leftMotor.configFactoryDefault();

        this.rightMotor.setInverted(true);
    }


    public void getSmartDashboardValues() {
        SmartDashboard.putNumber("Velocity", (this.rightMotor.getSelectedSensorVelocity() / RobotMap.DRIVETRAIN.VELOCITY_CONVERSION + 
            this.leftMotor.getSelectedSensorVelocity() / RobotMap.DRIVETRAIN.VELOCITY_CONVERSION) / 2);
        SmartDashboard.putNumber("Right Enc Vel.", this.rightMotor.getSelectedSensorVelocity() / RobotMap.DRIVETRAIN.VELOCITY_CONVERSION);
        SmartDashboard.putNumber("Left Enc Vel.", this.leftMotor.getSelectedSensorVelocity() / RobotMap.DRIVETRAIN.VELOCITY_CONVERSION);
    }
}
