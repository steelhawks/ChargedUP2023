// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer robot;
  private Command autonCommand;
  private Timer timer;
  private ColorSensorV3 colorSensor;

  // private RevColorSensorV3 revColorSensor;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Initialize Robot (Robot Container)
    this.colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    this.timer = new Timer();
    this.robot = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //4pl: y = 1.658006 + (21076420 - 1.658006)/(1 + (x/0.00476134)^1.509449)
    //5pl: y = 1.658006 + (145.6417 - 1.658006)/(1 + (x/12.58427)^9.809596)^0.1538747
    // System.out.println(
    //   (21076420 - 1.658006) /
    //   Math.pow((1 + (colorSensor.getProximity() / 0.00476134)), 1.509449)
    // );
    // System.out.println(
    //   (145.6417 - 1.658006) /
    //   Math.pow(
    //     1 + Math.pow((colorSensor.getProximity() / 12.58427), 9.809596),
    //     0.1538747
    //   )
    // );
    // System.out.println(
    //   (4288.3069 - 10.375) /
    //   (1 + Math.pow(colorSensor.getProximity() / 7.027, 1.4889))
    // );
    // System.out.println(
    //   5.3874 +
    //   (3419.831 - 5.3874) /
    //   (1 + Math.pow(colorSensor.getProximity() / 8.8838, 1.509449))
    // );
    // System.out.println(
    //   2.9385 +
    //   (2798.5671 - 2.9385) /
    //   (1 + Math.pow(colorSensor.getProximity() / 6.5398, 1.3882))
    // );
    // System.out.println(
    //   2.9354 +
    //   (2947.8981 - 2.9354) /
    //   (1 + Math.pow(colorSensor.getProximity() / 7.1801, 1.4563))
    // );
    System.out.println(
      7.4814 +
      (3365.1329 - 7.4814) /
      (1 + Math.pow(colorSensor.getProximity() / 6.7177, 1.4908))
    );
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    this.autonCommand = this.robot.autonChooser.getSelected();

    if (this.autonCommand != null) {
      autonCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (autonCommand != null) {
      autonCommand.cancel();
    }
    this.timer.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    this.robot.getDashboard();
    this.robot.endgameLED(this.timer.get());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
