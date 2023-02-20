package frc.robot;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.*;
// import frc.robot.commands.*;
import frc.robot.commands.Drive;


public class CommandLinker {
      /*****
   * Joystick Objects
   *****/

  public final Joystick joystick = new Joystick(ButtonMap.joystickOnePort);
  

  public CommandLinker() {
    
  }

  
  public void configureCommands()
  {
    CommandScheduler.getInstance().registerSubsystem(Robot.DRIVETRAIN);

    // Trigger SHIFT_BUTTON = new JoystickButton(this.driveJoystick, Robot.ROBOTMAP.getShiftButton());

     CommandScheduler.getInstance().setDefaultCommand(Robot.DRIVETRAIN, new Drive(Robot.DRIVETRAIN, Robot.COMMAND_LINKER.joystick));
  }
}
