package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    public final Joystick joystick;
    public final Drivetrain drivetrain;

    public RobotContainer() {
        this.joystick = new Joystick(0);
        this.drivetrain = new Drivetrain();

        // Register Subsystems
        CommandScheduler.getInstance().registerSubsystem(this.drivetrain);

        // Set Commands
        CommandScheduler.getInstance().setDefaultCommand(this.drivetrain,
                new ArcadeDrive(this.drivetrain, this.joystick));
    }
}