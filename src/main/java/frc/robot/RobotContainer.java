package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LED.SetColor;
import frc.robot.commands.LED.Wave;
import frc.robot.commands.auton.DefaultAuton;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.elevator.RaiseElevator;
import frc.robot.commands.elevator.LowerClaw;
import frc.robot.commands.elevator.RaiseClaw;
import frc.robot.commands.elevator.LowerElevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class RobotContainer {
    private final Joystick joystick;

    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Claw claw;
    private final LED LED;

    public final SendableChooser<Command> autonChooser = new SendableChooser<>();

    public RobotContainer() {
        this.joystick = new Joystick(0);

        this.drivetrain = new Drivetrain();
        this.elevator = new Elevator();
        this.claw = new Claw();
        this.LED = new LED();

        configAutonDashboard();
        configureCommands();
    }

    private void configureCommands() {
        this.LED.register();
        this.claw.register();
        this.drivetrain.register();
        this.elevator.register();

        this.drivetrain.setDefaultCommand(new ArcadeDrive(this.drivetrain, this.joystick));

        new JoystickButton(this.joystick, 5).onTrue(new SetColor(this.LED, new LEDColor(0, 255, 0)));
        new JoystickButton(this.joystick, 6).onTrue(new Wave(this.LED, 10, new LEDColor(0, 255, 0)));
        new JoystickButton(this.joystick, 7).onTrue(new RaiseElevator(this.elevator));
        new JoystickButton(this.joystick, 8).onTrue(new LowerElevator(this.elevator));
        new JoystickButton(this.joystick, 9).whileTrue(new RaiseClaw(this.elevator));
        new JoystickButton(this.joystick, 10).whileTrue(new LowerClaw(this.elevator));
        new JoystickButton(this.joystick, 11).onTrue(new OpenClaw(this.claw));
        new JoystickButton(this.joystick, 12).onTrue(new CloseClaw(this.claw));
    }

    public void configAutonDashboard() {
        autonChooser.addOption("No Auton", null);
        autonChooser.addOption("Default Auton", new DefaultAuton(this.drivetrain));

        SmartDashboard.putData("Autonomous Modes", autonChooser);
    }

    public void getSmartDashboardValues() {
        this.drivetrain.getSmartDashboardValues();
    }

}