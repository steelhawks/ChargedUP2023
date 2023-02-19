package frc.robot;

import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.util.ElevatorLevels;
import frc.lib.util.LEDColor;
import frc.lib.util.LEDMode;
import frc.robot.autos.*;
import frc.robot.commands.ToggleClaw;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Led.LedCommand;
import frc.robot.commands.Led.Request;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroCumulativeGyros = new JoystickButton(driver, 2); // A
    private final JoystickButton zeroGyro = new JoystickButton(driver, 3); // B
    private final JoystickButton robotCentric = new JoystickButton(driver, 5); // Left Bumper
    private final JoystickButton autoBalance = new JoystickButton(driver, 1); // X
    private final JoystickButton shiftGear = new JoystickButton(driver, 11); // Left Stick
    private final JoystickButton push = new JoystickButton(driver, 8); // Right trigger
    // private final POVButton upButton = new POVButton(driver, 0);
    // private final POVButton rightButton = new POVButton(driver, 90);
    // private final POVButton downButton = new POVButton(driver, 180);
    // private final POVButton leftButton = new POVButton(driver, 270);

    /*Operator Buttons*/ 
    private final JoystickButton homeElevator = new JoystickButton(operator, 2); // A
    private final JoystickButton lowElevator = new JoystickButton(operator, 3); // B
    private final JoystickButton midElevator = new JoystickButton(operator, 4); // Y
    private final JoystickButton highElevator = new JoystickButton(operator, 6); // Right bumper
    private final POVButton raiseElevator = new POVButton(operator, 0); // Up
    private final POVButton lowerElevator = new POVButton(operator, 180); // Down
    private final JoystickButton toggleClaw = new JoystickButton(operator, 5); // Left bumper
    private final JoystickButton toggleElevator = new JoystickButton(operator, 1); // X
    private final JoystickButton requestCone = new JoystickButton(operator, 7); // Left trigger
    private final JoystickButton requestCube = new JoystickButton(operator, 8); //Right trigger
    private final JoystickButton doubleSubButtion = new JoystickButton(operator, 10); //start button

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final StationPusher s_Pusher = new StationPusher();
    public static final Elevator s_Elevator = new Elevator();
    public static final LED s_Led = new LED(Constants.Led.port, Constants.Led.length);
    public static final Claw s_Claw = new Claw();
    public static final Vision s_Vision = new Vision();
    public static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    public RobotContainer() {
        // compressor.disable();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> driver.getPOV()
            )
        );

        CameraServer.startAutomaticCapture();

        configureButtonBindings();
    }

    private static Command elevatorLevelCommand(LEDColor color, ElevatorLevels level) {
        Command com = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(level),
                new LedCommand(color, LEDMode.STATIC)
            ),
            new ParallelRaceGroup(
                new LedCommand(color, LEDMode.PULSE),
                new WaitCommand(0.7)
            ),
            new LedCommand(LEDColor.OFF, LEDMode.STATIC)
        );

        return com;
    }

    // private static Command elevatorRainbowDoubleSubCommand() {
    //     Command com = new Command();
    //     return Command();
    // }
    
    private static Command ejectGamePieceCommmand() {
        Command com = new SequentialCommandGroup(
            new InstantCommand(() -> s_Claw.toggleClaw()),
            new WaitCommand(1),
            new InstantCommand(() -> s_Claw.toggleClaw())
        );
        return com;
    }

    private static Command requestPieceCommand(LEDColor color){
        Command com = new SequentialCommandGroup(new Request(color),
        new ToggleClaw(),
        new ParallelRaceGroup(
          new LedCommand(LEDColor.GREEN, LEDMode.PULSE),
          new WaitCommand(1)
        ),
        new LedCommand(LEDColor.OFF, LEDMode.STATIC)
      );
      return com;
    }

    public void configureMoreButtonBindings(int num) {
        zeroCumulativeGyros.onTrue(new InstantCommand(() -> s_Swerve.resetCumulativeModules(num)));
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        shiftGear.onTrue(new InstantCommand(() -> s_Swerve.shiftGear()));
        autoBalance.whileTrue(new BalanceCommand());
        // upButton.whileTrue(new RotateToAngle(0));
        // rightButton.whileTrue(new RotateToAngle(90));
        // downButton.whileTrue(new RotateToAngle(180));
        // leftButton.whileTrue(new RotateToAngle(270));
        push.onTrue(new InstantCommand(() -> s_Pusher.togglePusher()));

        /* Operator Buttons */
        homeElevator.onTrue(elevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME).andThen(new InstantCommand(() -> s_Claw.openClaw())));
        lowElevator.onTrue(elevatorLevelCommand(LEDColor.CYAN, ElevatorLevels.LOW));
        midElevator.onTrue(elevatorLevelCommand(LEDColor.BLUE, ElevatorLevels.MID));
        highElevator.onTrue(elevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH));
        raiseElevator.whileTrue(new ElevatorManual(true));
        lowerElevator.whileTrue(new ElevatorManual(false));
        toggleClaw.onTrue(new ToggleClaw());
        toggleElevator.onTrue(new ToggleElevator());
        requestCone.onTrue(requestPieceCommand(LEDColor.YELLOW));
        requestCube.onTrue(requestPieceCommand(LEDColor.PURPLE));
        // doubleSubButtion.onTrue(elevatorLevelCommand());
    }

    public Command getAutonomousCommand() {
        return new exampleAuto(s_Swerve);
    }
}
