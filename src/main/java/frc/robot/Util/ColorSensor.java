package frc.robot.Util;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatch;

import com.revrobotics.ColorMatchResult;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;


public class ColorSensor {

    ShuffleboardTab tab;
    private NetworkTableEntry red;
    private NetworkTableEntry green;
    private NetworkTableEntry blue;
    private NetworkTableEntry confidence;
    private NetworkTableEntry colorfound;


    //color sensor testing
    private final I2C.Port i2cPort;
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    
    // private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    // private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    // private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    // private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private final Color kBlueTarget = Color.kBlue;
    private final Color kGreenTarget = Color.kGreen;
    private final Color kRedTarget = Color.kRed;
    private final Color kYellowTarget = Color.kYellow;

    private Color detectedColor;
    private ColorMatchResult match;
    private String colorString;
    

    //constructor
    public ColorSensor(){

      this.tab = Shuffleboard.getTab("colors");

      // this.red = tab.add("Red", 0).getEntry();
      // this.green = tab.add("Green", 0).getEntry();
      // this.blue = tab.add("Blue", 0).getEntry();
      // this.confidence = tab.add("Confidence", 0).getEntry();
      // this.colorfound = tab.add("Detected Color", "").getEntry();

      this.i2cPort = I2C.Port.kOnboard;
      this.m_colorSensor = new ColorSensorV3(i2cPort);
      this.m_colorMatcher = new ColorMatch();
    }

    public void matchfixedcolors(){
            
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget); 
        
    }

    public Color detectColor(){
        this.detectedColor = m_colorSensor.getColor();

        
        this.match = m_colorMatcher.matchClosestColor(detectedColor);
    
        if (match.color == kBlueTarget) {
          colorString = "Blue";
        } else if (match.color == kRedTarget) {
          colorString = "Red";
        } else if (match.color == kGreenTarget) {
          colorString = "Green";
        } else if (match.color == kYellowTarget) {
          colorString = "Yellow";
        } else {
          colorString = "Unknown";
        }

        return detectedColor;
    }
    
    public void updatecolortable(){

      colorfound.setDefaultString("??????");

      red.setDouble(detectedColor.red);
      green.setDouble(detectedColor.green);
      blue.setDouble(detectedColor.blue);
      confidence.setDouble(match.confidence);
      colorfound.setString(colorString);
    }

    }



