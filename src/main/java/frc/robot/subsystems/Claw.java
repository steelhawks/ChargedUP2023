package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private DoubleSolenoid clawPiston; 
  private boolean isClosed;
  private boolean requestClose;
  private int breakCount = 0;
  private DigitalInput beamBreaker;
  private boolean hasCone;
  
  private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

  public Claw() {
    clawPiston = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidForward, Constants.Claw.SolenoidReverse);
    beamBreaker = new DigitalInput(Constants.Claw.beamPort);
    hasCone = !beamBreaker.get();

    isClosed = true;
    requestClose = true;
  }

  public void toggleClaw() {
    if (clawPiston.get().equals(Value.kReverse)) {
      closeClaw(false);
    }
    else {
      openClaw(false);
    }
  }

  public void closeClaw(boolean auto) {
    // if try to close automatically via beam breaker and claw is set to manually open
    if (auto && requestClose) return;

    clawPiston.set(Value.kForward);
    isClosed = true;
    requestClose = true;
  }

  public void openClaw(boolean auto) {
    // if try to open automatically via beam breaker and claw is set to manually close
    if (auto && !requestClose) return;

    clawPiston.set(Value.kReverse);
    isClosed = false;
    requestClose = false;
  }

  public DigitalInput getBeam() {
    return beamBreaker;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claw Beam", beamBreaker.get());
    SmartDashboard.putBoolean("Claw Closed", isClosed);
    // SmartDashboard.putBoolean("Has Piece", hasCone);
    
    // if (!beamBreaker.get()) {   // if the beam is broken
    //   breakCount++;             // increase breakCount by 1
    // }
    // else {                      // elif the beam is solid
    //   breakCount = 0;           // reset breakCount
    // }

    // if(beamBreaker.get() && hasCone) {
    //   hasCone = false;
    // }

    // if (!beamBreaker.get() && !isClosed && breakCount > 20 && !hasCone) {
    //   closeClaw();
    //   hasCone = true;
    // }


  }

  public boolean isClosed() {
    return isClosed;
  }

  public boolean hasCone() {
    return hasCone;
  }

  public void gotCone() {
    hasCone = true;
  }

  public void droppedCone() {
    hasCone = true;
  }
}
