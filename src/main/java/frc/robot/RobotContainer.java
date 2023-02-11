package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auton.DefaultAuton;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.elevator.Extend;
import frc.robot.commands.elevator.Retract;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    public final Joystick joystick;
    public final Drivetrain drivetrain;
    private final Elevator elevator;

    public final SendableChooser<Command> autonChooser = new SendableChooser<>();

    public RobotContainer() {
        this.joystick = new Joystick(0);
        this.drivetrain = new Drivetrain();
        this.elevator = new Elevator();

        configAutonDashboard();
        scheduleCommands();
    }

    public void scheduleCommands() {
        this.drivetrain.register();
        this.elevator.register();

        this.drivetrain.setDefaultCommand(new ArcadeDrive(this.drivetrain, this.joystick));

        new JoystickButton(this.joystick, 7).onTrue(new Extend(this.elevator));
        new JoystickButton(this.joystick, 8).onTrue(new Retract(this.elevator));
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