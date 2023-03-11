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
  private boolean isClosed = true;
  // private DoubleSolenoid clawPistonBottom; 
  public DigitalInput beamBreaker;
  

  private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

  public Claw() {
    clawPistonTop = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidTopForward, Constants.Claw.SolenoidTopReverse);
    // clawPistonBottom = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidBottomForward, Constants.Claw.SolenoidBottomReverse);
    beamBreaker = new DigitalInput(Constants.Claw.beamPort);
  }

  public void toggleClaw() {
    if (clawPistonTop.get().equals(Value.kReverse)) {
      closeClaw();
    }
    else {
      openClaw();
    }
  }

  public void closeClaw() {
    // clawPistonBottom.set(Value.kForward);
    clawPistonTop.set(Value.kForward);
    isClosed = false;
    
    // System.out.println("forward");

  }

  public void openClaw() {
    // clawPistonBottom.set(Value.kReverse);
    clawPistonTop.set(Value.kReverse);
    isClosed = true;
    
    // System.out.println("reverse");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Piece", beamBreaker.get());
    // Has game piece
    System.out.println(isClosed);
    if (!beamBreaker.get() && isClosed) {
      closeClaw();
      System.out.println("Close");
    }
  }
}
