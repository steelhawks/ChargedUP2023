package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  private DoubleSolenoid pistonOne;
  private WPI_TalonFX motorOne;
  private WPI_TalonFX motorTwo;
  private boolean pistonVal;

  private CANCoder can;
  private double initialRotations;

  private DigitalInput limitSwitchLow;
  private DigitalInput limitSwitchHigh;

  private static final PneumaticsModuleType PNEUMATIC_TYPE = PneumaticsModuleType.REVPH;

  public Elevator() {
    this.motorOne = new WPI_TalonFX(Constants.Elevator.motor1ID);
    this.motorTwo = new WPI_TalonFX(Constants.Elevator.motor2ID);

    this.pistonOne = new DoubleSolenoid(PNEUMATIC_TYPE, Constants.Elevator.SolenoidForward, Constants.Elevator.SolenoidReverse);

    can = new CANCoder(Constants.Elevator.canCoderID);
    initialRotations = getEncoderRotations();

    limitSwitchLow = new DigitalInput(Constants.Elevator.limitSwitchLowPort);
    limitSwitchHigh = new DigitalInput(Constants.Elevator.limitSwitchHighPort);

    pistonVal = false; // true is down

    // pistonsUp();
    configMotors();
    configCanCoders();
  }

  public void togglePistons() {
    if (pistonVal) {
      pistonsUp();
    } else if (getEncoderRotations() > Constants.Elevator.minPivotEncoderPos) {
      pistonsDown();
    }
    System.out.println("done");
  }

  public void pistonsUp() {
    pistonOne.set(Value.kForward);
    System.out.println("running piston up");
    pistonVal = false;

  }

  public void pistonsDown() {
    pistonOne.set(Value.kReverse);
    System.out.println("running piston down");
    pistonVal = true;
  }

  public void moveElevator(double speed, boolean isManual, boolean doubleSub) {
    double encoderVal = getEncoderRotations();
    boolean moveUp = speed < 0;

    if (moveUp && encoderVal < Constants.Elevator.maxEncoderPos) {
      if(!limitHighPressed()) stop();
      else {
        motorOne.set(speed);
        motorTwo.set(speed);
        if (!isManual && encoderVal >= Constants.Elevator.maxPivotEncoderPos && !pistonVal && !doubleSub) {
          // togglePistons();
          System.out.println("AUTO TOGGLE WIEFNWEJF");
        }
      }
    } else if (!moveUp && !limitLowPressed()) {
      motorOne.set(speed);
      motorTwo.set(speed);
    } else if (!moveUp && limitLowPressed()) {
      can.setPosition(0);
      stop();
    }

    if (!moveUp && encoderVal <= Constants.Elevator.minPivotEncoderPos && pistonVal) {
      pistonsUp();
      stop();
    }

    if (doubleSub && pistonVal) {
      pistonsUp();
    }
  }

  public double getEncoderRotations() {
    return (can.getPosition() - initialRotations) / 360;
  }

  public boolean limitLowPressed() {
    return !limitSwitchLow.get();
  }

  public boolean limitHighPressed() {
    return limitSwitchHigh.get();
  }

  private void configMotors() {
    motorOne.configFactoryDefault();
    motorOne.setNeutralMode(Constants.Elevator.motorNeutralMode);
    motorOne.getSensorCollection().setIntegratedSensorPosition(0, 0);
    motorOne.setInverted(true);

    motorTwo.configFactoryDefault();
    motorTwo.setNeutralMode(Constants.Elevator.motorNeutralMode);
    motorTwo.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  private void configCanCoders() {   
    can.configFactoryDefault();
    can.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    can.setPosition(0);
  }

  public void stop() {
    motorOne.stopMotor();
    motorTwo.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Values", getEncoderRotations());
    SmartDashboard.putBoolean("Elevator Low Limit", limitLowPressed());
    SmartDashboard.putBoolean("Elevator High Limit", limitHighPressed());
  }
}
