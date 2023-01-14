package frc.robot.Util; 

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;
//import java.lang.Object;
import java.util.ArrayList;

// import edu.wpi.first.math.trajectory.TrajectoryConfig; 
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.geometry.Translation2d;

public class PathTrajectory {
   
   String trajectoryJSON = "paths/PathtoBall1.wpilib.json";
   Trajectory trajectory = new Trajectory();
   
   //on robot init
   public void generateTrajectoryfromToolJson() {
      try {
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         trajectory =  TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
         DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
   }

   //alternative Path for PathtoBall1 but with startpoint and inbetween points
   public Trajectory generateTrajectory() {

      // waypoints for the path
      var startPose = new Pose2d(8.0, 2.0, Rotation2d.fromDegrees(Units.radiansToDegrees(-1.8076984698377099)));
      var endPose = new Pose2d(7.597159752741785, 0.43601290528415415, Rotation2d.fromDegrees(Units.radiansToDegrees(-1.8244704229334534)));
  
      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(7.858330195568157, 1.4305017544096224));
      interiorWaypoints.add(new Translation2d(7.718997481565451, 0.8962085028829527));
  
      TrajectoryConfig config = new TrajectoryConfig(6, 2);
  
      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          interiorWaypoints,
          endPose,
          config);

      return trajectory;
    }

}
