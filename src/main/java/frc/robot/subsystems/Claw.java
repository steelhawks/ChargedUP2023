package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private DoubleSolenoid clawPistonTop; 
  private DoubleSolenoid clawPistonBottom; 
  public DigitalInput beamBreaker;

  private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

  public Claw() {
    clawPistonTop = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidTopForward, Constants.Claw.SolenoidTopReverse);
    clawPistonBottom = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidBottomForward, Constants.Claw.SolenoidBottomReverse);
    beamBreaker = new DigitalInput(Constants.Claw.beamPort);
  }

  public void toggleClaw() {
    if (clawPistonBottom.get().equals(Value.kForward)){
      reverse();
    }
    else {
      forward();
    }
  }
  private void forward() {
    clawPistonBottom.set(Value.kForward);
    clawPistonTop.set(Value.kForward);
    // System.out.println("forward");

  }

  private void reverse() {
    clawPistonBottom.set(Value.kReverse);
    clawPistonTop.set(Value.kReverse);
    // System.out.println("reverse");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Piece", beamBreaker.get());
  }
}
