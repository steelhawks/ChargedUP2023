package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;
import java.util.ArrayList;

public class Wave extends CommandBase {

  private LED LED;
  private ArrayList<LEDColor> colors;
  private int waveLength;

  public Wave(LED LED, int waveLength, ArrayList<LEDColor> colors) {
    this.LED = LED;
    this.waveLength = waveLength;
    this.colors = colors;

    addRequirements(LED);
  }

  @Override
  public void execute() {
    this.LED.wave(this.waveLength, this.colors);
  }

  @Override
  public void end(boolean interrupted) {
    this.LED.setColor(new LEDColor(0, 0, 0));
  }
}
