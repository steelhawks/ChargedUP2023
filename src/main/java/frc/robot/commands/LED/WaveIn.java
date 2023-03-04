package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class WaveIn extends CommandBase {

  private LED LED;
  private LEDColor color;

  public WaveIn(LED LED, LEDColor color) {
    this.LED = LED;
    this.color = color;

    addRequirements(LED);
  }

  @Override
  public void initialize() {
    this.LED.zeroWaveIndex();
  }

  @Override
  public void execute() {
    this.LED.waveIn(this.color);
  }

  @Override
  public void end(boolean interrupted) {
    this.LED.setColor(new LEDColor(0, 0, 0));
  }
}
