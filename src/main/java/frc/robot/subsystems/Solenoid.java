package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid extends SubsystemBase {
    private final PneumaticsModuleType type = PneumaticsModuleType.REVPH;

    public PneumaticHub pneumaticHub = new PneumaticHub(1);
    public DoubleSolenoid solenoidOne; //= pneumaticHub.makeDoubleSolenoid(4, 5);
    public DoubleSolenoid solenoidTwo; //= pneumaticHub.makeDoubleSolenoid(6, 7);

    
    public Solenoid(){
      solenoidOne = new DoubleSolenoid(type, 4, 5);
      solenoidTwo = new DoubleSolenoid(type, 6, 7);
      
      solenoidOne.set(DoubleSolenoid.Value.kOff);
      solenoidTwo.set(DoubleSolenoid.Value.kOff);
    }

    public void toggleSolenoid() {
        if (this.solenoidOne.get().equals(DoubleSolenoid.Value.kForward)) {
            retractSolenoid();
        } else {
            extendSolenoid();
        }
      }
    
      public void extendSolenoid() {
        this.solenoidOne.set(DoubleSolenoid.Value.kForward);
        this.solenoidTwo.set(DoubleSolenoid.Value.kForward);
      }
    
      public void retractSolenoid() {
        this.solenoidOne.set(DoubleSolenoid.Value.kReverse);
        this.solenoidTwo.set(DoubleSolenoid.Value.kReverse);
      }
}
