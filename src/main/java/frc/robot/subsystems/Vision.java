package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase{
    
    // Pipeline IDs'
    public static final int ID3_PIPELINE = 0;
    private static final int CONE_PIPELINE = 1;
    public static final int ID2_PIPELINE = 2;
    public static final int ID1_PIPELINE = 3;
    private static final int CUBE_PIPELINE = 4;
    private static final int REFLECTIVE_TAPE_PIPELINE = 5;

    public Vision(){
        Limelight.init();
    }

    public void goToTag(){
        if (Limelight.hasValidTarget() && Limelight.getArea() < Constants.Vision.areaThreshold) {
            double y_vel;
            double ang_vel;

            if (Math.abs(Limelight.getXOffset()) > Constants.Vision.xOffsetThreshold) {
              y_vel = (Math.sin((Math.PI / Constants.Vision.FiftyFour) * Limelight.getXOffset()));
            } else {
              y_vel = 0;
            }
      
            if(Math.abs(Limelight.getTagYaw()) < 0.15){
                ang_vel = 0;
            } else {
                ang_vel = -(Math.sin((Math.PI / 14) * Limelight.getTagYaw()));
            }

            Translation2d velocity = new Translation2d(Constants.Vision.xVelocity, y_vel);
            RobotContainer.s_Swerve.drive(velocity, ang_vel, true, false);
        } else if (!Limelight.hasValidTarget()) {
            RobotContainer.s_Swerve.drive(new Translation2d(0, 0), Constants.Vision.spinVelocity, true, false);
        }
    }

    public void squareToTag(){
        double ang_vel;

        if(Math.abs(Limelight.getTagYaw()) < 0.4) {
            ang_vel = 0;
        } else {
            ang_vel = -Math.sin((Math.PI / 14) * Limelight.getTagYaw());
        }

        RobotContainer.s_Swerve.drive(new Translation2d(0, 0), ang_vel, true, false);
    }

    public Command testTrajectory(Trajectory trajectory) {
        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
          SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              trajectory,
              RobotContainer.s_Swerve::getPose, // Functional interface to feed supplier
              Constants.Swerve.swerveKinematics,
    
              // Position controllers
              new PIDController(Constants.AutoConstants.kPController, 0, 0),
              new PIDController(Constants.AutoConstants.kPController, 0, 0),
              thetaController,
              RobotContainer.s_Swerve::setModuleStates,
              RobotContainer.s_Swerve);
            
          // return swerveControllerCommand;
          return new InstantCommand(() -> RobotContainer.s_Swerve.resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand);
      }
}
