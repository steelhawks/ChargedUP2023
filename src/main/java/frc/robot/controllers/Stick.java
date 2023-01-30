package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TestCommand;

public class Stick {
  public Joystick stick;

  public Stick() {
    this.stick = new Joystick(0);

    new JoystickButton(stick, 0).whileTrue(new TestCommand());
  }
}
