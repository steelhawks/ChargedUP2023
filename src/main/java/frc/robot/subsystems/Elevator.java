package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private DoubleSolenoid pistonOne;
  private DoubleSolenoid pistonTwo;
  private WPI_TalonFX motorOne;
  private WPI_TalonFX motorTwo;
  private boolean elevatorUp;

  private static final PneumaticsModuleType PNEUMATIC_TYPE = PneumaticsModuleType.CTREPCM;

  public Elevator() {
    this.motorOne = new WPI_TalonFX(Constants.Elevator.motor1ID);
    this.motorTwo = new WPI_TalonFX(Constants.Elevator.motor2ID);

    this.pistonOne = new DoubleSolenoid(PNEUMATIC_TYPE, Constants.Elevator.Solenoid1Forward, Constants.Elevator.Solenoid1Reverse);
    this.pistonTwo = new DoubleSolenoid(PNEUMATIC_TYPE, Constants.Elevator.Solenoid2Forward, Constants.Elevator.Solenoid2Reverse);

    elevatorUp = false;
    configMotors();
  }

  public void togglePistons() {
    if (!elevatorUp) {
      pistonOne.set(Value.kForward);
      pistonTwo.set(Value.kForward);
    } else if (getEncoderVal() > Constants.Elevator.minPivotEncoderPos) {
      pistonOne.set(Value.kReverse);
      pistonTwo.set(Value.kReverse);
    }

    elevatorUp = !elevatorUp;
  }

  public void moveElevator(boolean moveUp) {
    double encoderVal = getEncoderVal();

    if (moveUp && encoderVal < Constants.Elevator.maxEncoderPos) {
      motorOne.set(0.5);
      motorTwo.set(0.5);
    } else if (!moveUp && encoderVal > Constants.Elevator.minEncoderPos) {
      motorOne.set(-0.5);
      motorTwo.set(-0.5);
    }
  }

  // Move elevator to set encoder position
  public void moveToPosition(double speed) {
    motorOne.set(speed);
    motorTwo.set(speed);
  }

  public double getEncoderVal() {
    return motorOne.getSensorCollection().getIntegratedSensorPosition() / 2048;
  }

  private void configMotors() {
    motorOne.configFactoryDefault();
    motorOne.setNeutralMode(Constants.Elevator.motorNeutralMode);
    motorOne.getSensorCollection().setIntegratedSensorPosition(0, 0);

    motorTwo.configFactoryDefault();
    motorTwo.setNeutralMode(Constants.Elevator.motorNeutralMode);
    motorTwo.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public void stop() {
    motorOne.stopMotor();
    motorTwo.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Values", getEncoderVal());
  }
}
