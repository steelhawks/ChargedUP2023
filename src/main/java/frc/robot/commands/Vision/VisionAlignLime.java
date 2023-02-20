package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class VisionAlignLime extends CommandBase {
  
  public VisionAlignLime() {
    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {

  }
  
  @Override
  public void execute() {
    
    RobotContainer.s_Vision.goToTag();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("findinsb go to 3");
    RobotContainer.s_Swerve.stop();

  }

  @Override
  public boolean isFinished() {
    return Limelight.getArea() > Constants.Vision.areaThreshold;
  }
}
