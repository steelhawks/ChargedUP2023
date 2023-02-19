package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StationPusher extends SubsystemBase {

  private DoubleSolenoid pusher;
  private boolean isExtended;

  public StationPusher() {
    pusher = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pusher.solenoidForward, Constants.Pusher.solenoidReverse);
    isExtended = false;
  }

  public void extend() {
    if (!isExtended) {
      pusher.set(DoubleSolenoid.Value.kForward);
      isExtended = true;
    }
  }

  public void retract() {
    if (isExtended){
      pusher.set(DoubleSolenoid.Value.kReverse);
      isExtended = false;
    }
  }

  public void togglePusher() {
    if(isExtended){
      retract();
      System.out.println("retract");
    }
    else{
      System.out.println("extend");
      extend();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Extended", isExtended);
  }
}
