package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.I2C;


public class Claw extends SubsystemBase {

  private DoubleSolenoid clawPiston; 
  private boolean isClosed;
  private boolean requestClose;
  private DigitalInput beamBreaker;

  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatch;

  private static final Color CONE_COLOR = new Color(0.361, 0.524, 0.113);
  private static final Color CUBE_COLOR = new Color(0.245, 0.411, 0.343);
  
  private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

  public Claw() {
    clawPiston = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Constants.Claw.SolenoidForward, Constants.Claw.SolenoidReverse);
    beamBreaker = new DigitalInput(Constants.Claw.beamPort);

    this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    this.colorMatch = new ColorMatch();
    this.colorMatch.addColorMatch(CUBE_COLOR);
    this.colorMatch.addColorMatch(CONE_COLOR);

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

    //  double x = 7.4814 +
    //   (3365.1329 - 7.4814) /
    //   (1 + Math.pow(colorSensor.getProximity() / 6.7177, 1.4908)); 

    SmartDashboard.putNumber("color sensor", colorSensor.getProximity()); 
      // System.out.println(colorSensor.getProximity());
    SmartDashboard.putString("color", colorMatch.matchClosestColor(colorSensor.getColor()).color.toString());
  }

  public boolean isClosed() {
    return isClosed;
  }
}
