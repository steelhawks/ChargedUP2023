package frc.robot.commands.Led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.lib.util.LEDColor;
import frc.lib.util.LEDMode;

public class LedCommand extends CommandBase {

  private LEDColor color;
  private LEDMode mode;
  private final double pulseInterval = 0.1;

  public LedCommand(LEDColor color, LEDMode mode) {
    this.color = color;
    this.mode = mode;

    addRequirements(RobotContainer.s_Led);
  }

  @Override
  public void initialize() {
    if (mode != LEDMode.RAINBOW) {
      RobotContainer.s_Led.setColor(color);
    }
  }

  @Override
  public void execute() {

    if (mode == LEDMode.PULSE) {
      RobotContainer.s_Led.pulse(color, pulseInterval);
    }
    else if (mode == LEDMode.STATIC) {
      RobotContainer.s_Led.setColor(color);

    }
    else if (mode == LEDMode.RAINBOW) {
      RobotContainer.s_Led.rainbow();
    }
    else if (mode == LEDMode.WAVE) {
      RobotContainer.s_Led.wave(color);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (mode == LEDMode.STATIC) {
      return true;
    }
    else {
      return false;
    }
  }
}
