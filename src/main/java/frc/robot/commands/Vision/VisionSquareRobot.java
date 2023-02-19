package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.RobotContainer;

public class VisionSquareRobot extends CommandBase {
  
  public VisionSquareRobot() {
    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RobotContainer.s_Vision.squareToTag();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(Limelight.getTagYaw()) < 0.4;
  }
}
