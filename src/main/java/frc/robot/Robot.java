/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class Robot extends TimedRobot 
{
  /*****
   * Robot Objects
   *****/
  public static final RobotMap ROBOTMAP = new RobotMap();
  public static final Drivetrain DRIVETRAIN = new Drivetrain();
  public static final CommandLinker COMMAND_LINKER = new CommandLinker();
  public double count = -0.99;

  //ultrasonic
  final AnalogInput ultrasonic = new AnalogInput(0);
  

  @Override
  public void robotInit() 
  {
    COMMAND_LINKER.configureCommands();

    //CommandScheduler.getInstance().registerSubsystem(Robot.DRIVETRAIN);
    //CommandScheduler.getInstance().registerSubsystem(Robot.ULTRA);
    //CommandScheduler.getInstance().registerSubsystem(Robot.VISION);

    //Button SHIFT_BUTTON = new JoystickButton(Robot.COMMAND_LINKER.DRIVE_JOYSTICK, Robot.ROBOTMAP.getShiftButton());
    //Button ALIGN_BUTTON = new JoystickButton(Robot.COMMAND_LINKER.DRIVE_JOYSTICK, Robot.ROBOTMAP.getAlignButton());

    //SHIFT_BUTTON.whenPressed(new ShiftGear());
    //ALIGN_BUTTON.whenPressed(new Align());

    //colorsensortest

    //tab.add("currentDistanceCentimeters", currentDistanceCentimeters).getEntry();
    //tab.add("currentDistanceInches", currentDistanceInches).getEntry();
    
    //ultrasonic testing


  }

  @Override
  public void robotPeriodic() 
  {    
    double raw_value = ultrasonic.getValue();
    //voltage_scale_factor allows us to compensate for differences in supply voltage.
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    double currentDistanceCentimeters = raw_value * voltage_scale_factor * 0.125;
    double currentDistanceInches = raw_value * voltage_scale_factor * 0.0492;
    System.out.println(currentDistanceCentimeters + "cm");
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    CommandScheduler.getInstance().enable();


    
  }
  public void teleopPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {}


}
