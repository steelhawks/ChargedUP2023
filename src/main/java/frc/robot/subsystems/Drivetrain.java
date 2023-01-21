package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.kauailabs.navx.frc.AHRS;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap;
import frc.robot.subsystems.*;


public class Drivetrain {
    // LEFT MOTORS
    public final WPI_TalonSRX LEFT_MOTOR_ONE;
    public final WPI_TalonSRX LEFT_MOTOR_TWO;
    public final WPI_TalonSRX LEFT_MOTOR_THREE;
    
    // RIGHT MOTOR
    public final WPI_TalonSRX RIGHT_MOTOR_ONE;
    public final WPI_TalonSRX RIGHT_MOTOR_TWO;
    public final WPI_TalonSRX RIGHT_MOTOR_THREE;
  
    // public final DifferentialDrive diffDrive;


  
  
    //@Override
    public boolean stop(){
      // TODO Auto-generaated method stub
      return false;
    }
    //@Override
    /** Pings the subsystem. */
   public void ping(){
      // TODO Auto-generaated method stub
  
    }
    // @Override
    /** Checks if the subsystem is functioning properly. @return True if functioning */
    public boolean isAlive(){
      // TODO Auto-generaated method stub
      return false;
    }
    //@Override
    /** Print info to shuffleboard */
    public void shuffleBoard(){
      // TODO Auto-generaated method stub
  
      
    }
    
    public Drivetrain() 
    {
      //SPARK MAX LEFT MOTORS
      this.LEFT_MOTOR_ONE = new WPI_TalonSRX(Robot.ROBOTMAP.getdrivetrainLeftMotorPortOne());
      this.LEFT_MOTOR_TWO = new WPI_TalonSRX(Robot.ROBOTMAP.getdrivetrainLeftMotorPortTwo());
      this.LEFT_MOTOR_THREE = new WPI_TalonSRX(Robot.ROBOTMAP.getdrivetrainLeftMotorPortThree());
      
      //SPARK MAX RIGHT MOTORS
      this.RIGHT_MOTOR_ONE = new WPI_TalonSRX(Robot.ROBOTMAP.getdrivetrainRightMotorPortOne());
      this.RIGHT_MOTOR_TWO = new WPI_TalonSRX(Robot.ROBOTMAP.getdrivetrainRightMotorPortTwo());
      this.RIGHT_MOTOR_THREE = new WPI_TalonSRX(Robot.ROBOTMAP.getdrivetrainRightMotorPortThree());
    }
    public void configureMotors() {
      this.LEFT_MOTOR_ONE.configFactoryDefault();
      this.LEFT_MOTOR_TWO.configFactoryDefault();
      this.LEFT_MOTOR_THREE.configFactoryDefault(); 
      this.RIGHT_MOTOR_ONE.configFactoryDefault();
      this.RIGHT_MOTOR_TWO.configFactoryDefault();
      this.RIGHT_MOTOR_THREE.configFactoryDefault();
      this.LEFT_MOTOR_ONE.setNeutralMode(NeutralMode.Coast);
      this.LEFT_MOTOR_TWO.setNeutralMode(NeutralMode.Coast);
      this.LEFT_MOTOR_THREE.setNeutralMode(NeutralMode.Coast);
      this.RIGHT_MOTOR_ONE.setNeutralMode(NeutralMode.Coast);
      this.RIGHT_MOTOR_TWO.setNeutralMode(NeutralMode.Coast);
      this.RIGHT_MOTOR_TWO.setNeutralMode(NeutralMode.Coast);
    }
}

