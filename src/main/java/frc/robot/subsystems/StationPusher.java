// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StationPusher extends SubsystemBase {

  private DoubleSolenoid pusher;
  private boolean isExtended;

  public StationPusher() {
    pusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    isExtended = false;
  }

  public void push(){
    if (!isExtended) {
      pusher.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retract(){
    if(isExtended){
      pusher.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void togglePusher(){
    if(isExtended){
      retract();
    }
    else{
      push();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Extended", isExtended);
  }
}
