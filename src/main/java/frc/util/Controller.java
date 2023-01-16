package frc.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller{

  private final Joystick joystick;

  public Controller(Joystick joystick) {
    this.joystick = joystick;
  }

  public JoystickButton mapButton(int buttonNumber){
    return new JoystickButton(this.joystick, buttonNumber);
  }

}