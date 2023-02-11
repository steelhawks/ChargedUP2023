// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private DoubleSolenoid rightSolenoid;
  private DoubleSolenoid leftSolenoid;

  public Elevator() {
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

    rightSolenoid.set(DoubleSolenoid.Value.kOff);
    leftSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void extend() {
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}