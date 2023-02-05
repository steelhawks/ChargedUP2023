package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auton.DefaultAuton;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    public final Joystick joystick;
    public final Drivetrain drivetrain;

    public final SendableChooser<Command> autonChooser = new SendableChooser<>();

    public RobotContainer() {
        this.joystick = new Joystick(0);
        this.drivetrain = new Drivetrain();

        configAutonDashboard();

        // Register Subsystems
        CommandScheduler.getInstance().registerSubsystem(this.drivetrain);

        // Set Commands
        CommandScheduler.getInstance().setDefaultCommand(this.drivetrain,
                new ArcadeDrive(this.drivetrain, this.joystick));

    }

    public void getSmartDashboardValues() {
        this.drivetrain.getSmartDashboardValues();
    }

    public void configAutonDashboard() {
        autonChooser.addOption("No Auton", null);
        autonChooser.addOption("Default Auton", new DefaultAuton(this.drivetrain));

        SmartDashboard.putData("Autonomous Modes", autonChooser);
    }
}