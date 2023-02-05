// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StationPusher extends SubsystemBase {
  
  private DoubleSolenoid pusher;

  public StationPusher() {
    pusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }

  public void push(boolean extend){
    if (extend) {
      pusher.set(DoubleSolenoid.Value.kForward);
    }
    else {
      pusher.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
