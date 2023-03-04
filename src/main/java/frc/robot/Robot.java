// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.revrobotics.AnalogInput;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Solenoid;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // public static Subsystem Drivetrain;
  public static final RobotMap ROBOTMAP = new RobotMap();
  public static final Drivetrain DRIVETRAIN = new Drivetrain();
  public static final CommandLinker COMMAND_LINKER = new CommandLinker();
  public static final Solenoid SOLENOID = new Solenoid();


  // private static final Sendable gyro = null;
  private Command m_autonomousCommand;
  // private final AnalogInput ultrasonic = new AnalogInput(0);
  private RobotContainer m_robotContainer;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kPurpleTarget = new Color(0.245, 0.411, 0.343);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  private static DigitalInput Button = new DigitalInput(2);

  private DigitalOutput initialBeamBreaker = new DigitalOutput(3);
  private DigitalInput finalBeamBreaker = new DigitalInput(4);

  private Counter pololuDistSensor = new Counter(Counter.Mode.kPulseLength);
  private PWM pololuPWM = new PWM(0);
  // private AnalogInput pololuSensor = new AnalogInput(0);

  // private AnalogInput sharp = new AnalogInput(1);
  
  // Pigeon2 _pigeon = new Pigeon2(0, "rio");
    // int _loopCount = 0;
    // private Ultrasonic ultrasonic; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Shuffleboard.getTab("Example tab").add(gyro);

    // ultrasonic = new Ultrasonic(1,0); // trig, echo
    // ultrasonic.setAutomaticMode(true);
    // ultrasonic.setEnabled(true);

    // System.out.println(ultrasonic.getRangeInches());
  

    m_colorMatcher.addColorMatch(kPurpleTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    COMMAND_LINKER.configureCommands();

    
    // pololuDistSensor.setUpSource(0);
    // pololuDistSensor.setUpSourceEdge(true, true);
    // pololuDistSensor.setPulseLengthMode(.05);
    pololuDistSensor.reset();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Intake Proximity", proximity);
    
    // SmartDashboard.putNumber("double ultrasonic", ultrasonic.getRangeInches());
    
    double pololu = pololuDistSensor.getDistance();
    double p = pololuDistSensor.getRate(); 
    double d = pololuDistSensor.getPeriod();
    boolean h = pololuDistSensor.getDirection(); 
    System.out.println("rate"+p); 
    System.out.println("direction"+h); 
    System.out.println(pololu); 
    System.out.println(d);
    SmartDashboard.putNumber("pololu (mm)", d);

  
    double x = pololuPWM.getPosition();
    System.out.println(x);

    // int x = pololuSensor.getValue();
    // long y = pololuSensor.getAccumulatorCount(); 
    // System.out.println((x- 1000) * 4);
    // SmartDashboard.putNumber("pololu dist?", (x- 1000) * 4);
    // System.out.println(x); 
    // System.out.println(y); 


      if (match.color == kPurpleTarget && match.confidence >= 0.97) {
      colorString = "Cube";
    } else if (match.color == kYellowTarget && match.confidence >= 0.97) {
      colorString = "Cone";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putString("Detected Color", colorString);

  }

 
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // Set setpoint to current heading at start of auto
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // DigitalInput toplimitSwitch = new DigitalInput(0);
  // DigitalInput bottomlimitSwitch = new DigitalInput(1);
  // PWMVictorSPX motor = new PWMVictorSPX(0);
  // Joystick joystick = new Joystick(0);


//   public void setMotorSpeed(double speed) {
//     if (speed > 0) {
//         if (toplimitSwitch.get()) {
//             // We are going up and top limit is tripped so stop
//             motor.set(0);
//         } else {
//             // We are going up but top limit is not tripped so go at commanded speed
//             motor.set(speed);
//         }
//     } else {
//         if (bottomlimitSwitch.get()) {
//             // We are going down and bottom limit is tripped so stop
//             motor.set(0);
//         } else {
//             // We are going down but bottom limit is not tripped so go at commanded speed
//             motor.set(speed);
//         }
//     }


// }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // setMotorSpeed(joystick.getRawAxis(2));

    /**
     * double rawValue = ultrasonic.getValue();
    
    //voltage_scale_factor allows us to compensate for differences in supply voltage.
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    double currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
    double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;
  
    System.out.println(currentDistanceCentimeters + currentDistanceInches);
    */
    

    //gyro
    // if(_loopCount++ > 10)
    // {
    //     _loopCount = 0;
    //     double yaw = _pigeon.getYaw();
    //     System.out.println("Pigeon Yaw is: " + yaw);
    // }
     boolean button = Button.get();
     SmartDashboard.putBoolean("Button", !button); 
     // button gives opposite values so use ! 

     // beam breaker
     SmartDashboard.putBoolean("Send Beam", !initialBeamBreaker.get());
     SmartDashboard.putBoolean("Beam Break?", !finalBeamBreaker.get());

     // double sharpDist = Math.pow(sharp.getAverageVoltage(), -1.2045) * 27.726;
     // SmartDashboard.putNumber("Distance (sharp)", sharpDist);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
