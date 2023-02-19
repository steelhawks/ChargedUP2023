package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    private static NetworkTableInstance tableInstance;
    private static NetworkTable table;

    private static double ta; // target area
    private static double tx; // x offset or horizontal angle fromcenter
    private static double ty; // y offset
    private static double tv; // if target present 1; if not 0
    private static double horizontalBoundingSide; // horizontal side of rough bounding box
    private static double verticalBoundingSide; // vertical side of rough bounding box
    private static double[] contours;
    private static double[] apriltag_pose; 
    private static double[] robot_space;

    private static int pipeline;

    public static void init(){
        tableInstance = NetworkTableInstance.getDefault();
        table = tableInstance.getTable("limelight");
        pipeline = 0;
    }

    public static void updateValues(){

        ta = getEntryAsDouble("ta");
        tx = getEntryAsDouble("tx");
        ty = getEntryAsDouble("ty");
        tv = getEntryAsDouble("tv");
        apriltag_pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[0]);
        robot_space = table.getEntry("botpose").getDoubleArray(new double[0]);
        horizontalBoundingSide = getEntryAsDouble("thor");
        verticalBoundingSide = getEntryAsDouble("tvert");
        
        pipeline = (int)getEntryAsDouble("getpipe");

        contours = table.getEntry("tcornxy").getDoubleArray(new double[0]);
    }

    public static double getArea(){
        return ta;
    }

    public static double getXOffset(){
        return tx;
    }

    public static double getYOffset(){
        return ty;
    }

    public static boolean hasValidTarget(){
        return tv == 1;
    }

    public static double getHorizontalSide(){
        return horizontalBoundingSide;
    }

    public static double getVerticalSide(){
        return verticalBoundingSide;
    }

    public static double[] getContours(){
        return contours;
    }

    public static double getTagYaw(){
        return apriltag_pose[5];
    }

    public static Pose2d getApriltagPose() {
        return new Pose2d(apriltag_pose[0], apriltag_pose[1], new Rotation2d(apriltag_pose[5])); 
    }

    public static Pose2d getRobotSpace() {
        return new Pose2d(robot_space[0], robot_space[1], new Rotation2d(robot_space[5]));
    }

    public static int getPipeline(){
        return pipeline;
    }

    public static void setPipeline(int pipeline){
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public static double getEntryAsDouble(String key){
        return table.getEntry(key).getDouble(0);
    }


}