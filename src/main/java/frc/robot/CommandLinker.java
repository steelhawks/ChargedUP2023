package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class CommandLinker {
      /*****
   * Joystick Objects
   *****/

  public final Joystick driveJoystick = new Joystick(ButtonMap.joystickOnePort);
  

  public CommandLinker() {
    //configureCommands();
  }

  
  public void configureCommands()
  {
    CommandScheduler.getInstance().registerSubsystem(Robot.Drivetrain);

    Trigger SHIFT_BUTTON = new JoystickButton(this.driveJoystick, Robot.ROBOTMAP.getShiftButton());

    // CommandScheduler.getInstance().setDefaultCommand(Robot.Drivetrain, new diffDrive());
  }
}
