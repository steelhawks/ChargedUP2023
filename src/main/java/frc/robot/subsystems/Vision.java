package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.Limelight;
// import frc.lib.util.LimelightTrajectory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase{
    
    // Pipeline IDs'
    private static final int CONE_PIPELINE = 1;
    private static final int BLUE_SUBSTATION_PIPELINE = 2;
    private static final int RED_SUBSTATION_PIPELINE = 3;
    private static final int CUBE_PIPELINE = 4;
    private static final int REFLECTIVE_TAPE_PIPELINE = 5;

    private static PIDController controller;
    // public final LimelightTrajectory sathya = new LimelightTrajectory();

    // public Trajectory traj = sathya.generateTargetTrajectory(Robot.config);

    
    public Vision(){
        Limelight.init();

    }

    public void GoToTag(int pipeline){
        // RobotContainer.s_Swerve.rotateToAngle(180);
        Limelight.setPipeline(pipeline);

        if (Limelight.hasValidTarget() && Limelight.getArea() < Constants.Vision.areaThreshold) {
            double y_vel;
            if (Math.abs(Limelight.getXOffset()) > Constants.Vision.xOffsetThreshold) {
              y_vel = (Math.sin(Math.PI / Constants.Vision.FiftyFour) * Limelight.getXOffset());
            } 
            
            else {
              y_vel = 0;
            }
      
            Translation2d velocity = new Translation2d(0, y_vel);
            if (RobotContainer.s_Swerve.isLowGear()) {
                velocity = velocity.times(5);
            }
            RobotContainer.s_Swerve.drive(velocity, 0, true, false);
        } 
        else if (!Limelight.hasValidTarget()) {
            RobotContainer.s_Swerve.stop();
        }
        
        // else if (!Limelight.hasValidTarget()) {
        //     RobotContainer.s_Swerve.drive(new Translation2d(-.5, 0), 0, true, false);
        // }
    }

    public void goLeft() {
        Trajectory goL = TrajectoryGenerator.generateTrajectory(
            RobotContainer.s_Swerve.getPose(),
            List.of(),
            new Pose2d(0, 5, new Rotation2d(0)),
            Robot.config
        );

        // return loadCommand(goL);
    }

    public void goRight() {
        RobotContainer.s_Swerve.drive(new Translation2d(0, -Constants.Vision.NodeDistance), 0, true, false);
        
    }



    public void squareToTag() {
        RobotContainer.s_Swerve.rotateToAngle((int)Limelight.getXOffset());
    }

    public Command goodCommand() {
        System.out.println("helo");
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(), 
        new Pose2d(0,1, new Rotation2d(0)), 
        new TrajectoryConfig(0.5, 0.5).setKinematics(Constants.Swerve.swerveKinematics));
        
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
          trajectory, // PUT TRAJECTORY HERE
          RobotContainer.s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          new PIDController(1, 0, 0),
          new PIDController(1, 0, 0),
          thetaController,
          RobotContainer.s_Swerve::setModuleStates,
          RobotContainer.s_Swerve);
            
          RobotContainer.s_Swerve.resetOdometry(trajectory.getInitialPose());

          return swerveControllerCommand.andThen(() -> RobotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false));
      }

    public void scheduleCommand(){
        this.goodCommand().schedule();
    }

}
