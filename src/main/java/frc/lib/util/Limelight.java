package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
        apriltag_pose = table.getEntry("botpose").getDoubleArray(new double[0]); 
        horizontalBoundingSide = getEntryAsDouble("thor");
        verticalBoundingSide = getEntryAsDouble("tvert");
        
        pipeline = (int)getEntryAsDouble("getpipe");

        contours = table.getEntry("tcornxy").getDoubleArray(new double[0]);

        // TODO fix entry key
        apriltag_pose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
    }

    public static double getArea(){
        return getEntryAsDouble("ta");
    }

    public static double getXOffset(){
        return getEntryAsDouble("tx");
    }

    public static double getYOffset(){
        return getEntryAsDouble("ty");
    }

    public static boolean hasValidTarget(){
        return getEntryAsDouble("tv") == 1;
    }

    public static double getHorizontalSide(){
        return getEntryAsDouble("thor");
    }

    public static double getVerticalSide(){
        return getEntryAsDouble("tvert");
    }

    public static double[] getContours(){
        return table.getEntry("tcornxy").getDoubleArray(new double[0]);
    }

    public static double[] getAprilTagPose(){
        return apriltag_pose;
    }

    // public static Pose2d getApriltagPose() {
    //     return new Pose2d(apriltag_pose[0], apriltag_pose[1], new Rotation2d(apriltag_pose[2])); 
    // }

    public static int getPipeline(){
        return (int)getEntryAsDouble("getpipe");
    }

    public static void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public static double getEntryAsDouble(String key) {
        return table.getEntry(key).getDouble(0);
    }
}