package frc.robot.commands.Led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.lib.util.LEDColor;
import frc.lib.util.LEDMode;

public class LedClaw extends CommandBase {

  public LedClaw() {

    addRequirements(RobotContainer.s_Led);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(RobotContainer.s_Claw.isClosed()) RobotContainer.s_Led.setColor(LEDColor.RED);
    else RobotContainer.s_Led.setColor(LEDColor.GREEN);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
