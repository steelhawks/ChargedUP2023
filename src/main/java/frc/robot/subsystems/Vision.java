package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.lib.util.LimelightTrajectory;
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
    public final LimelightTrajectory sathya = new LimelightTrajectory();

    public Trajectory traj = sathya.generateTargetTrajectory(Robot.config);


    
    public Vision(){
        Limelight.init();
    }

    public void GoToTag(){
        if (Limelight.hasValidTarget() && Limelight.getArea() < Constants.Vision.areaThreshold) {
            double y_vel;
            if (Math.abs(Limelight.getXOffset()) > Constants.Vision.xOffsetThreshold) {
              y_vel = -(Math.sin(Math.PI / Constants.Vision.FiftyFour) * Limelight.getXOffset());
            } else {
              y_vel = 0;
            }
      
            Translation2d velocity = new Translation2d(Constants.Vision.xVelocity, y_vel);
            RobotContainer.s_Swerve.drive(velocity, 0, true, false);
        } else if (!Limelight.hasValidTarget()) {
            RobotContainer.s_Swerve.drive(new Translation2d(0, 0), Constants.Vision.spinVelocity, true, false);
        }
    }

    public void squareToTag(){
        
    }

    public Trajectory getSathya() {
        if(traj == null) return TrajectoryGenerator.generateTrajectory(
                // robot pose -> target space 
                new Pose2d(0,0, new Rotation2d(0)),
                // Pass through no interior points 
                List.of(),
                // End at apriltag pose 
                new Pose2d(0, 1, new Rotation2d(0)),
                Robot.config);
        else return traj;
    }
}
