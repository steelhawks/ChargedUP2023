package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.LEDColor;
import frc.lib.util.Limelight;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;
  private SendableChooser<Integer> moduleChooser;
  private SendableChooser<Integer> moduleCumulativeChooser;

  private RobotContainer m_robotContainer;

  public static TrajectoryConfig config;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
 
    moduleChooser = new SendableChooser<>();
    moduleCumulativeChooser = new SendableChooser<>();

    config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.Swerve.swerveKinematics);

      moduleChooser.setDefaultOption("None", -1);
      moduleChooser.addOption("Module 0", 0);
      moduleChooser.addOption("Module 1", 1);
      moduleChooser.addOption("Module 2",2);
      moduleChooser.addOption("Module 3",3);
      
      moduleCumulativeChooser.setDefaultOption("1 Module", 1);
      moduleCumulativeChooser.addOption("2 Modules", 2);
      moduleCumulativeChooser.addOption("3 Modules", 3);
      moduleCumulativeChooser.addOption("4 Modules", 4);
      
      SmartDashboard.putData(moduleChooser);
      SmartDashboard.putData(moduleCumulativeChooser);
    
      m_robotContainer = new RobotContainer();
      SmartDashboard.putData(m_robotContainer.getAutonChooser());
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Limelight.updateValues();
    SmartDashboard.putNumber("Pressure", RobotContainer.compressor.getPressure());
  }

  @Override
  public void disabledInit() {
    RobotContainer.s_Led.setColor(LEDColor.RED);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putString("Chosen Auton", m_robotContainer.getAutonName());
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.configureModuleResetBinding(moduleCumulativeChooser.getSelected());
    RobotContainer.s_Led.setColor(LEDColor.OFF);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
