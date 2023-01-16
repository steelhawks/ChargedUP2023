package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class Drivetrain extends SubsystemBase{
    // MOTORS
  public final WPI_TalonSRX LEFT_MOTOR_ONE;
  public final WPI_TalonSRX RIGHT_MOTOR_ONE;

  public final MotorControllerGroup leftGroup;
  public final MotorControllerGroup rightGroup;

  //DIFFERENTIAL DRIVE
  public final DifferentialDrive DIFF_DRIVE;
  public boolean isForward;

  //VARIABLE RPM ELECTRO-SHIFT
  public int shiftStatus;
  public double rPMCoefficient;

  //TWIST COEFFICIENT
  public double twistCoefficient;

  //NAVX MXP GYRO
  public final Pigeon2 GYRO;
  public final double KP_GYRO;

  public Drivetrain() 
  {
    //SPARK MAX LEFT MOTORS
    this.LEFT_MOTOR_ONE = new WPI_TalonSRX(1);
    //SPARK MAX RIGHT MOTORS
    this.RIGHT_MOTOR_ONE = new WPI_TalonSRX(0);

    //SPEED CONTROLLER GROUPS
    this.rightGroup = new MotorControllerGroup(this.LEFT_MOTOR_ONE);
    this.leftGroup = new MotorControllerGroup(this.LEFT_MOTOR_ONE);

    //DIFFERENTIAL DRIVE
    this.DIFF_DRIVE = new DifferentialDrive(this.rightGroup, this.leftGroup);

    //NAVX MXP GYRO
    this.GYRO = new Pigeon2(5);
    this.KP_GYRO = 0;

    //VARIABLE RPM ELECTRO-SHIFT
    this.rPMCoefficient = 1.25;//original 1.75

    //TWIST COEFFICIENT
    this.twistCoefficient = 1.25;

    Constants.Drivetrain.LEFT.configure(LEFT_MOTOR_ONE);
    Constants.Drivetrain.RIGHT.configure(RIGHT_MOTOR_ONE);
    
  }

  //DRIVING METHOD
  public void arcadeDrive(Joystick stick) 
  {
    double y = stick.getY();
    double rotate = stick.getTwist();
    this.DIFF_DRIVE.arcadeDrive(y / this.rPMCoefficient, rotate / this.twistCoefficient, false);

  }
  
  //MOVING STRAIGHT USING THE GYRO METHOD
  public void gyroMoveStraight(double speed)
  {
    this.DIFF_DRIVE.arcadeDrive(speed, -this.GYRO.getYaw() * this.KP_GYRO);
  }

  //MOVING STRAIGHT USING GYRO AND ANGLE VALUE METHOD
  public void gyroMoveStraight(double speed, double angle)
  {
    this.DIFF_DRIVE.arcadeDrive(-speed, -angle * this.KP_GYRO);
  }

  //ROTATE ROBOT
  public void rotate(double speed)
  {
    this.leftGroup.set(speed);
    this.rightGroup.set(speed);
  }

  public Pigeon2 getGyro()
  {
    return this.GYRO;
  }

  public double getGyroAngle() 
  {
    return this.GYRO.getYaw(); 
  }

  public ErrorCode getAllAngles(double[] ypr_deg) 
  {
    return this.GYRO.getYawPitchRoll(ypr_deg);
  }

  public void resetGyro() 
  {
    this.GYRO.setYaw(0);
  }

  public boolean stop(){
    rotate(0);
    return false;
  }

  /** Pings the subsystem. */
  public void ping(){}

  /** Checks if the subsystem is functioning properly. @return True if functioning */
  public boolean isAlive(){
    return true;
  }

  /** Print info to shuffleboard */
  public void shuffleBoard(){
    SmartDashboard.putNumber("L1 velocity", this.LEFT_MOTOR_ONE.getSensorCollection().getQuadratureVelocity());
    SmartDashboard.putNumber("R1 velocity", this.RIGHT_MOTOR_ONE.getSensorCollection().getQuadratureVelocity());

    SmartDashboard.putNumber("TalonFXControlMode.Velocity", TalonFXControlMode.Velocity.value);
    SmartDashboard.putNumber("TalonFXControlMode.PercentOutput", TalonFXControlMode.PercentOutput.value);
    SmartDashboard.putNumber("TalonFXControlMode.Position", TalonFXControlMode.Position.value);


    SmartDashboard.putNumber("gyroangle", getGyroAngle());
    
  }
  
  //VISION STUFF

  // LOOK FOR BALL
  public void spinRobot() {
    this.DIFF_DRIVE.arcadeDrive(0, 0.7);
  }

  
  // Turns robot 180 degrees
  private void rotateRobot() {
    this.DIFF_DRIVE.arcadeDrive(0, 180);
  }



}