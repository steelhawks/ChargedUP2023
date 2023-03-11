package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LED.SetColor;
import frc.robot.commands.LED.Wave;
import frc.robot.commands.LED.WaveIn;
import frc.robot.commands.auton.DefaultAuton;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.DetectPiece;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.elevator.LowerClaw;
import frc.robot.commands.elevator.LowerElevator;
import frc.robot.commands.elevator.RaiseClaw;
import frc.robot.commands.elevator.RaiseElevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class RobotContainer {

  private final Joystick joystick;
  private final Joystick LEDJoystick;

  public final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Claw claw;
  private final LED LED;

  public final SendableChooser<Command> autonChooser;

  public RobotContainer() {
    this.autonChooser = new SendableChooser<>();

    this.joystick = new Joystick(0);
    this.LEDJoystick = new Joystick(1);

    this.drivetrain = new Drivetrain();
    this.elevator = new Elevator();
    this.claw = new Claw();
    this.LED = new LED(60);

    configAutonDashboard();
    configureCommands();
  }

  private void configureCommands() {
    this.LED.register();
    this.claw.register();
    this.drivetrain.register();
    this.elevator.register();

    this.drivetrain.setDefaultCommand(
        new ArcadeDrive(this.drivetrain, this.joystick)
      );
    this.claw.setDefaultCommand(new DetectPiece(this.claw));

    new JoystickButton(this.LEDJoystick, 1)
      .onTrue(new SetColor(this.LED, new LEDColor(0, 0, 0)));
    new JoystickButton(this.LEDJoystick, 2)
      .onTrue(new SetColor(this.LED, new LEDColor(0, 255, 0)));
    new JoystickButton(this.LEDJoystick, 3)
      .onTrue(
        new Wave(
          this.LED,
          30,
          new LEDColor[] {
            new LEDColor(0, 255, 0),
            new LEDColor(0, 0, 255),
            new LEDColor(255, 0, 0),
          }
        )
      );
    new JoystickButton(this.LEDJoystick, 4)
      .onTrue(new WaveIn(this.LED, new LEDColor(0, 255, 0)));
    new JoystickButton(this.LEDJoystick, 5)
      .onTrue(new SetColor(this.LED, new LEDColor(0, 0, 255)));
    new JoystickButton(this.joystick, 7)
      .onTrue(new RaiseElevator(this.elevator));
    new JoystickButton(this.joystick, 8)
      .onTrue(new LowerElevator(this.elevator));
    new JoystickButton(this.joystick, 9)
      .whileTrue(new RaiseClaw(this.elevator));
    new JoystickButton(this.joystick, 10)
      .whileTrue(new LowerClaw(this.elevator));
    new JoystickButton(this.joystick, 11).onTrue(new OpenClaw(this.claw));
    new JoystickButton(this.joystick, 12).onTrue(new CloseClaw(this.claw));
  }

  public void configAutonDashboard() {
    this.autonChooser.addOption("No Auton", null);
    this.autonChooser.addOption(
        "Default Auton",
        new DefaultAuton(this.drivetrain, this.LED)
      );

    SmartDashboard.putData("Autonomous Modes", autonChooser);
  }

  public void getDashboard() {
    this.drivetrain.getDashboard();
  }

  public void endgameLED(double timer) {
    this.LED.endgameLED(timer);
  }
}
