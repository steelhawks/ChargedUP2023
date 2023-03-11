package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class SetColor extends CommandBase {

  private LED LED;
  private LEDColor color;

  public SetColor(LED LED, LEDColor color) {
    this.LED = LED;
    this.color = color;

    addRequirements(LED);
  }

  @Override
  public void execute() {
    this.LED.setColor(color);
  }
}
