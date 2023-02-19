package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid solenoid;

  public Claw() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.CLAW.SOLENOID_FORWARD, RobotMap.CLAW.SOLENOID_REVERSE);
  }

  public void openClaw() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
