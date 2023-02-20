// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static interface Field{
    static double imperialToMeters(int feet, double inches){
      return Units.feetToMeters(feet) + Units.inchesToMeters(inches);
    }

    double Width = imperialToMeters(26, 3.5);
    double Length = imperialToMeters(54, 3.5);
    Translation2d Center = new Translation2d(0, 0);

    public interface Cone{
      double ConeHeight = imperialToMeters(1, 0.8125);
      double ConeBaseWidth = imperialToMeters(0, 8.375);
      double ConeBaseRadius = imperialToMeters(0, 3.3125);
      double ConeTopRadius = imperialToMeters(0, 0.875);
      // how to calculate width of cone depending on cone position and height of grab
    }

    public interface Cube{
      double CubeWidth = imperialToMeters(0, 9.5);
      // +/- .25 in (deviation of .25 in from the width depending on cube position)
      // width is measured from face to face aka. max width
    }

    public interface Grids{
      //location

      //measurements
      double GridLengthTotal = imperialToMeters(18, 0.5);
      double GridLengthCOOP = imperialToMeters(5, 6);
      double GridLengthOuter = imperialToMeters(6, 3);
      double GridWidth = imperialToMeters(4, 6.25);

      // cone node 
      // height respective to the field carpet
      double ConeNodeRadius = imperialToMeters(0, .83);

      double TopNodeHeight = imperialToMeters(3, 10);
      double TopNodeDist = imperialToMeters(3, 3.75);

      double MidNodeHeight = imperialToMeters(2, 10);
      double MidNodeDist = imperialToMeters(1, 10.75);

      // cube node / shelves
      // dist is distance away from the face of the grid
      double TopShelfHeight = imperialToMeters(2, 11.5);
      double TopShelfDist = imperialToMeters(2, 7.625);

      double MidShelfHeight = imperialToMeters(1, 11.5);
      double MidShelfDist = imperialToMeters(1, 2.25);

      // hybrid node
      double HybridDist = imperialToMeters(1, 4);
      double HybridWidth = imperialToMeters(1, 6.2);
      double OuterHybridWidth = imperialToMeters(2, 1.75);
      double HybridDividerHeight = imperialToMeters(0, 5);
    }

    public interface ChargeStation{
      double ChargeStationLevelHeight = imperialToMeters(0, 9.125);

      double ChargeStationPlatformWidth = imperialToMeters(8, 0);
      double ChargeStationPlatformLength = imperialToMeters(4, 0);
      double ChargeStationBorderWidth = imperialToMeters(8, 1.25);
      double ChargeStationBorderLength = imperialToMeters(6, 4.125);
    }


  }
}
