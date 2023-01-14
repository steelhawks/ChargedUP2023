package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;

public class CommandLinker 
{
  /*****
   * Joystick Objects
   *****/
  public final Joystick DRIVE_JOYSTICK = new Joystick(Robot.ROBOTMAP.getJoystickPortOne());

  public CommandLinker() 
  {
    //configureCommands();
  }

  public void configureCommands()
  {
    CommandScheduler.getInstance().registerSubsystem(Robot.DRIVETRAIN);

    Trigger SHIFT_BUTTON = new JoystickButton(this.DRIVE_JOYSTICK, Robot.ROBOTMAP.getShiftButton());
    // SHIFT_BUTTON.whenPressed(new ShiftGear());


    CommandScheduler.getInstance().setDefaultCommand(Robot.DRIVETRAIN, new DiffDrive());
  }
}
