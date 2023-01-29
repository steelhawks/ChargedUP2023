package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    public final Joystick joystick = new Joystick(0); 
    public final Drivetrain drivetrain = new Drivetrain();   
}
