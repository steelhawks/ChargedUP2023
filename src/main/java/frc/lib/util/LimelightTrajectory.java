package frc.lib.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LimelightTrajectory {

    //create NetworkTable objects
    NetworkTableInstance nInstance;
    NetworkTable table; 
    
    //limelight values
    private NetworkTableEntry ta; 
    private NetworkTableEntry tv; 
    private NetworkTableEntry ty;
    private NetworkTableEntry tx; 
    private NetworkTableEntry tid; 
    private NetworkTableEntry tagpose;
    private NetworkTableEntry botpose; 
    private int pipeline; 
    private Trajectory trajectory;  

    //trajectory fields
    HashMap<Double, Pose2d> poses; 

    public LimelightTrajectory() {
        System.out.println("Limelight object initialized");
        
        nInstance = NetworkTableInstance.getDefault();
        table = nInstance.getTable("limelight");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tid = table.getEntry("tid");
        tagpose = table.getEntry("targetpose_camerapose");
        botpose = table.getEntry("camerapose_targetspace");
        poses = new HashMap<Double, Pose2d>(); 
        trajectory = null; 
        this.setPipeline(0);
    
    }

    public Pose2d getTagPose() {
        double[] botposeEntry = botpose.getDoubleArray(new double[6]);
        //double[] tagposeEntry = tagpose.getDoubleArray(new double[6]);  
        return new Pose2d(botposeEntry[2], botposeEntry[1], new Rotation2d(0)); //if going backwards, negate whatever the pose values are 
        
        //botposeEntry[2] -> z 
        //botposeEntry[1] -> Y

        /* The z coordinate of the limelight coordinate system represents the x coordinate of the actual field
         * The y coordinate of the limelight coordinate system represents the y coordinate of the actual field 
         * The actual logic should negative the limelight z and y values because we are using robot space. 
         * However negating them makes the robot move backwards. 
         */


    }

    public void printTargetPoses() {
        System.out.println(Arrays.asList(poses));
    }

    public double getTa() {
        return ta.getDouble(0); 
    }

    public double getTv() {
        return tv.getDouble(0); 
    }

    public double getTx() {
        return tx.getDouble(0); 
    }

    public double getTy() {
        return ty.getDouble(0); 
    }

    public double getTid() {
        return tid.getDouble(0); 
    }

    public String getPipeline() {
        return "Currently using pipeline " + pipeline; 
    }

    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
        table.getEntry("pipeline").setNumber(pipeline);
        System.out.println("Pipeline changed to " + pipeline); 
    } 

    public void mapOriginPairs() {
        //initialize dictionary 
        if (this.getTv() == 1) {
            poses.put(this.getTid(), this.getTagPose());
            System.out.println("Successfully mapped");
        }

        else {
            System.out.println("No mapping to be done");
        }

    }

    public Trajectory generateTargetTrajectory(TrajectoryConfig config) {
       
        System.out.println("resetting trajectory relative to target pose");
         //adding relative coordinate pathing for now delete later 
        System.out.println("Trajectory generated successfully"); 
        RobotContainer.s_Swerve.resetOdometry(RobotContainer.s_Swerve.getPose());

        if (this.getTv() == 1) {
            
            System.out.println("Valid target exists"); 

            //RobotContainer.s_Swerve.resetOdometry(RobotContainer.s_Swerve.getPose()); //sets origin to tag pose 
            trajectory = TrajectoryGenerator.generateTrajectory(
                // robot pose -> target space 
                new Pose2d(0,0, new Rotation2d(0)),
                // Pass through no interior points 
                List.of(),
                // End at apriltag pose 
                this.getTagPose(),
                config);

        }
        
        
        return trajectory;
    
    }


}