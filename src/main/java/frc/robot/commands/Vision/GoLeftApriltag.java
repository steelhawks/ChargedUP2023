package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GoLeftApriltag extends CommandBase {

  public GoLeftApriltag() {
    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Vision.goLeft();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
