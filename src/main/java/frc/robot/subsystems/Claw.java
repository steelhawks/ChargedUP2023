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
  private int breakCount = 0;
  // private DoubleSolenoid clawPistonBottom; 
  public DigitalInput beamBreaker;
  public boolean hasCone;
  

  private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

  public Claw() {
    clawPistonTop = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidTopForward, Constants.Claw.SolenoidTopReverse);
    // clawPistonBottom = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidBottomForward, Constants.Claw.SolenoidBottomReverse);
    beamBreaker = new DigitalInput(Constants.Claw.beamPort);
    hasCone = !beamBreaker.get();
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
    isClosed = true;
    
    // System.out.println("forward");

  }

  public void openClaw() {
    // clawPistonBottom.set(Value.kReverse);
    clawPistonTop.set(Value.kReverse);
    isClosed = false;
    
    // System.out.println("reverse");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam", beamBreaker.get());
    SmartDashboard.putBoolean("Has Piece", hasCone);
    // Has game piece
    // System.out.println(isClosed);
    // System.out.println(breakCount);
    if (!beamBreaker.get()) {   // if the beam is broken
      breakCount++;             // increase breakCount by 1
    }
    else {                      // elif the beam is solid
      breakCount = 0;           // reset breakCount
    }

    if(beamBreaker.get() && hasCone) hasCone = false;

    if (!beamBreaker.get() && !isClosed && breakCount > 20 && !hasCone) {
      closeClaw();
      hasCone = true;
    }

    

 

    
    // if (!beamBreaker.get() && !isClosed) {
    //   closeClaw();
    //   System.out.println("Close");
    // }

    // If beam broken and the claw is 
    // Doesn't work because 
  }
}
