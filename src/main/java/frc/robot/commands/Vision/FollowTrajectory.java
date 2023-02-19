package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.RobotContainer;

public class FollowTrajectory extends CommandBase {
  
  public FollowTrajectory() {
    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RobotContainer.s_Vision.GoToTag();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return Limelight.getArea() > 1.6;
  }
}
