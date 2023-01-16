package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.DrivetrainDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.util.Controller;
import frc.util.Gamepad;

public class RobotContainer {

    public final Joystick driver = new Joystick(0);
    public final Gamepad operatorGamepad = new Gamepad(1);
    
    public final Controller operator = new Controller(operatorGamepad);

    //Subsystem
    public final Drivetrain drivetrain = new Drivetrain();

    public RobotContainer() {

        configureDefaultCommands();
    }


    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainDriveCommand(drivetrain, driver));
    }
}
