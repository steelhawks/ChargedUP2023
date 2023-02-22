// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {

  private DoubleSolenoid rightSolenoid;
  private DoubleSolenoid leftSolenoid;
  private WPI_TalonFX rightElevatorMotor;
  private WPI_TalonFX leftElevatorMotor;

  public Elevator() {
    this.rightSolenoid =
      new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        RobotMap.ELEVATOR.SOLENOID_RIGHT_FORWARD,
        RobotMap.ELEVATOR.SOLENOID_RIGHT_REVERSE
      );
    this.leftSolenoid =
      new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        RobotMap.ELEVATOR.SOLENOID_LEFT_FORWARD,
        RobotMap.ELEVATOR.SOLENOID_LEFT_REVERSE
      );
    this.rightElevatorMotor =
      new WPI_TalonFX(RobotMap.ELEVATOR.MOTOR_RIGHT_ONE_PORT);
    this.leftElevatorMotor =
      new WPI_TalonFX(RobotMap.ELEVATOR.MOTOR_RIGHT_ONE_PORT);

    this.rightSolenoid.set(DoubleSolenoid.Value.kOff);
    this.leftSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void extend() {
    this.rightSolenoid.set(DoubleSolenoid.Value.kForward);
    this.leftSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void raiseClaw() {
    this.rightElevatorMotor.set(0.5);
    this.leftElevatorMotor.set(0.5);
  }

  public void lowerClaw() {
    this.rightElevatorMotor.set(-0.5);
    this.leftElevatorMotor.set(-0.5);
  }
}
