package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.kauailabs.navx.frc.AHRS;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap;
import frc.robot.subsystems.*;
*/

public class Drivetrain extends SubsystemBase{
    // LEFT MOTORS
    public final WPI_TalonFX LEFT_MOTOR_ONE; 
    // public final WPI_TalonSRX LEFT_MOTOR_TWO;
    // public final WPI_TalonSRX LEFT_MOTOR_THREE;
    
    // RIGHT MOTOR
    public final WPI_TalonFX RIGHT_MOTOR_ONE; 
    // public final WPI_TalonSRX RIGHT_MOTOR_TWO;
    // // public final WPI_TalonSRX RIGHT_MOTOR_THREE;

    //DIFFERENTIAL DRIVE
    public final DifferentialDrive differentialDrive;

    //VARIABLE RPM ELECTRO-SHIFT
    public int shiftStatus;
    public double rPMCoefficient;

    public Drivetrain()
    {
      this.LEFT_MOTOR_ONE = new WPI_TalonFX(Robot.ROBOTMAP.getdrivetrainLeftMotorPortOne());
      this.RIGHT_MOTOR_ONE = new WPI_TalonFX(Robot.ROBOTMAP.getdrivetrainRightMotorPortOne());

      this.differentialDrive = new DifferentialDrive(RIGHT_MOTOR_ONE, LEFT_MOTOR_ONE);

          //VARIABLE RPM ELECTRO-SHIFT
      this.shiftStatus = 1;
      this.rPMCoefficient = 1.75;

      double pi = 3.141592653589793;
      double conversion = 10 / 2048 * 0.1524 * pi;
      double gearRatio = 1 / 12.75;
      configureMotors();

      LEFT_MOTOR_ONE.getSelectedSensorVelocity();
      RIGHT_MOTOR_ONE.getSelectedSensorVelocity();

      SmartDashboard.putNumber("LEFT SPEED", LEFT_MOTOR_ONE.getSelectedSensorVelocity());
      SmartDashboard.putNumber("RIGHT SPEED", RIGHT_MOTOR_ONE.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Velocity (m/s)", (LEFT_MOTOR_ONE.getSelectedSensorVelocity() + RIGHT_MOTOR_ONE.getSelectedSensorVelocity())/2 * (conversion * gearRatio));
    }
    
    //DRIVING METHOD
    public void arcadeDrive(Joystick stick) 
    {
      this.differentialDrive.arcadeDrive(stick.getY() / (5*this.rPMCoefficient), -stick.getTwist(), false);
    }
    
    //@Override
    public boolean stop(){
      return false;
    }

    //@Override
    /** Pings the subsystem. */
    public void ping(){}

    // @Override
    /** Checks if the subsystem is functioning properly. @return True if functioning */
    public boolean isAlive(){
      return false;
    }

    //@Override
    /** Print info to shuffleboard */
    public void shuffleBoard(){}
    
    

    public void configureMotors() {
      this.LEFT_MOTOR_ONE.configFactoryDefault();
      // this.LEFT_MOTOR_TWO.configFactoryDefault();
      // this.LEFT_MOTOR_THREE.configFactoryDefault(); 
      this.RIGHT_MOTOR_ONE.configFactoryDefault();
      // this.RIGHT_MOTOR_TWO.configFactoryDefault();
      // this.RIGHT_MOTOR_THREE.configFactoryDefault();


      this.LEFT_MOTOR_ONE.setNeutralMode(NeutralMode.Coast);
      // this.LEFT_MOTOR_TWO.setNeutralMode(NeutralMode.Coast);
      // this.LEFT_MOTOR_THREE.setNeutralMode(NeutralMode.Coast);
      this.RIGHT_MOTOR_ONE.setNeutralMode(NeutralMode.Coast);
      // this.RIGHT_MOTOR_TWO.setNeutralMode(NeutralMode.Coast);
      // this.RIGHT_MOTOR_TWO.setNeutralMode(NeutralMode.Coast);
    }

    public void arcadeDrive(double fwd, double rot) {
      differentialDrive.arcadeDrive(fwd, rot);
    }
  
    //@Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}

