package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class Wave extends CommandBase {

  private LED LED;
  private LEDColor[] colors;
  private int waveLength;

  public Wave(LED LED, int waveLength, LEDColor[] colors) {
    this.LED = LED;
    this.waveLength = waveLength;
    this.colors = colors;

    addRequirements(LED);
  }

  @Override
  public void initialize() {
    this.LED.zeroWaveIndex();
  }

  @Override
  public void execute() {
    this.LED.waveLoop(this.waveLength, this.colors);
  }
}
