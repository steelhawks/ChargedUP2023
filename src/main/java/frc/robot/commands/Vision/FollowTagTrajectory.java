package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.LimelightTrajectory;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FollowTagTrajectory extends CommandBase {

  LimelightTrajectory limelight = new LimelightTrajectory(); 
  TrajectoryConfig config;

  public FollowTagTrajectory() {
    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
     config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.Swerve.swerveKinematics);
    
    
      loadCommand(limelight.generateTargetTrajectory(config));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private Command loadCommand(Trajectory trajectory) {
    var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
      trajectory,
      RobotContainer.s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPController, Constants.AutoConstants.kIController, Constants.AutoConstants.kDController),
      new PIDController(Constants.AutoConstants.kPController, Constants.AutoConstants.kIController, Constants.AutoConstants.kDController),
      thetaController,
      RobotContainer.s_Swerve::setModuleStates,
      RobotContainer.s_Swerve);
        
      return swerveControllerCommand;
  }
}
