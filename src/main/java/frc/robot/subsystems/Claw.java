package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid solenoid;

  public Claw() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  }

  public void openClaw() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
