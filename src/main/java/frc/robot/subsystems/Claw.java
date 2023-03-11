package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {

  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatch;
  private final DoubleSolenoid solenoid;

  public Claw() {
    this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    this.colorMatch = new ColorMatch();
    this.colorMatch.addColorMatch(RobotMap.CLAW.CUBE_COLOR);
    this.colorMatch.addColorMatch(RobotMap.CLAW.CONE_COLOR);

    this.solenoid =
      new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        RobotMap.CLAW.SOLENOID_FORWARD,
        RobotMap.CLAW.SOLENOID_REVERSE
      );
  }

  public void detectPiece() {
    System.out.println(
      7.4814 +
      (3365.1329 - 7.4814) /
      (1 + Math.pow(colorSensor.getProximity() / 6.7177, 1.4908))
    );
    ColorMatchResult match = colorMatch.matchClosestColor(
      colorSensor.getColor()
    );

    if (
      match.color == RobotMap.CLAW.CUBE_COLOR ||
      match.color == RobotMap.CLAW.CONE_COLOR &&
      match.confidence >= 0.97
    ) closeClaw(); else openClaw();
  }

  public void openClaw() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
