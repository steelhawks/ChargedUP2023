package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AlignType;
import frc.lib.util.ElevatorLevels;
import frc.lib.util.GamepadAxisButton;
import frc.lib.util.LEDColor;
import frc.lib.util.LEDMode;
import frc.robot.commands.Claw.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Led.*;
import frc.robot.commands.Vision.VisionAlignLime;

import frc.robot.subsystems.*;

public class RobotContainer {

    /* Auton Chooser */
    private SendableChooser<Command> autonChooser = new SendableChooser<>();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton aButton = new JoystickButton(driver, 2); // A
    private final JoystickButton bButton = new JoystickButton(driver, 3); // B
    private final JoystickButton xButton = new JoystickButton(driver, 1); // X
    private final JoystickButton leftStick = new JoystickButton(driver, 11); // Left Stick
    private final JoystickButton rightTrigger = new JoystickButton(driver, 8); // Right trigger
    private final JoystickButton leftTrigger = new JoystickButton(driver, 7); // Left trigger
    private final JoystickButton robotCentric = new JoystickButton(driver, 5); // Left Bumper
    private final JoystickButton rightBumper = new JoystickButton(driver, 6); // Right Bumper

    /* Operator Button Board Buttons */
    private final JoystickButton homeElevator = new JoystickButton(operator, 5);
    private final JoystickButton lowElevator = new JoystickButton(operator, 3);
    private final JoystickButton midElevator = new JoystickButton(operator, 2);
    private final JoystickButton highElevator = new JoystickButton(operator, 1);
    private final JoystickButton singleSubButton = new JoystickButton(operator, 11);
    private final JoystickButton toggleClaw = new JoystickButton(operator, 4);
    private final JoystickButton doubleSubButton = new JoystickButton(operator, 6);
    private final JoystickButton requestCone = new JoystickButton(operator, 7);
    private final JoystickButton requestCube = new JoystickButton(operator, 8);
    private final JoystickButton elevatorPivot = new JoystickButton(operator, 10);
    private final GamepadAxisButton raiseElevator = new GamepadAxisButton(() -> operator.getRawAxis(1) == -1); // Up
    private final GamepadAxisButton lowerElevator = new GamepadAxisButton(() -> operator.getRawAxis(1) == 1); // Down

    /* Auton selector */
    private static final DigitalInput zero = new DigitalInput(10);
    private static final DigitalInput one = new DigitalInput(11);
    private static final DigitalInput two = new DigitalInput(12);
    private static final DigitalInput three = new DigitalInput(13);
    private static final DigitalInput four = new DigitalInput(18);
    private static final DigitalInput five = new DigitalInput(19);
    private static final DigitalInput six = new DigitalInput(20);
    private static final DigitalInput seven = new DigitalInput(21);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Elevator s_Elevator = new Elevator();
    public static final LED s_Led = new LED(Constants.Led.port, Constants.Led.length);
    public static final Claw s_Claw = new Claw();
    public static final Vision s_Vision = new Vision();
    public static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    // Claw Beam
    Trigger clawBeam = new Trigger(s_Claw.getBeam()::get);

    public RobotContainer() {
        // compressor.disable();
        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        CameraServer.startAutomaticCapture();
        configureButtonBindings();
        configureDefaultCommands();
        configureAutons();
        configureSensors();
    }

    /* Command to move the elevator to a given level */
    private static Command elevatorLevelCommand(LEDColor color, ElevatorLevels level) {
        Command com = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(level),
                new InstantCommand(() -> s_Elevator.pistonsDown()),
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

    /* Command to mot ethe elevator to a given level in AUTON */
    private static Command autoElevatorLevelCommand(LEDColor color, ElevatorLevels level) {
        Command com = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(level),
                new InstantCommand(() -> s_Elevator.pistonsDown()),
                new LedCommand(color, LEDMode.STATIC)
            ),
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new LedCommand(color, LEDMode.PULSE),
                    new WaitCommand(0.2)
                ),
                new InstantCommand(() -> s_Claw.openClaw(true))
            ),
            new LedCommand(LEDColor.OFF, LEDMode.STATIC)
        );

        return com;
    }

    // private static Command requestPieceCommand(LEDColor color){
    //     return new SequentialCommandGroup(new Request(color),
    //     new ToggleClaw(),
    //     new ParallelRaceGroup(
    //       new LedCommand(LEDColor.GREEN, LEDMode.PULSE),
    //       new WaitCommand(1)
    //     ),
    //     new LedCommand(LEDColor.OFF, LEDMode.STATIC)
    //   );
    // }

    public void configureModuleResetBinding(int num) {
        aButton.onTrue(new InstantCommand(() -> s_Swerve.resetCumulativeModules(num)));
    }

    private void configureButtonBindings() {

        /* Driver Buttons */
        bButton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        leftStick.onTrue(new InstantCommand(() -> s_Swerve.shiftGear()));
        xButton.whileTrue(new BalanceCommand());
        rightTrigger.onTrue(new InstantCommand(() -> s_Swerve.shiftGear()));
        leftTrigger.whileTrue(new NodeAlign(AlignType.CONE));

         //align to imaginary point left of tag
        // leftTrigger.whileTrue(new SequentialCommandGroup(
        //     new VisionAlignLime(3) //point to left of tag is pipeline 0
        // ));

        //align to imaginary point right of tag 
        // rightBumper.whileTrue(new SequentialCommandGroup(
        //     new VisionAlignLime(0) //point to the right of tag is pipeline 3
        // ));

        /* Operator Buttons */
        homeElevator.onTrue(new InstantCommand(() -> s_Claw.openClaw(true)).andThen(elevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME)));
        lowElevator.onTrue(elevatorLevelCommand(LEDColor.CYAN, ElevatorLevels.LOW));
        midElevator.onTrue(elevatorLevelCommand(LEDColor.BLUE, ElevatorLevels.MID));
        highElevator.onTrue(elevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH));
        doubleSubButton.onTrue(elevatorLevelCommand(LEDColor.ORANGE, ElevatorLevels.DOUBLE_STATION));
        singleSubButton.onTrue(elevatorLevelCommand(LEDColor.CYAN, ElevatorLevels.SINGLE_STATION));

        raiseElevator.whileTrue(new ElevatorManual(true));
        lowerElevator.whileTrue(new ElevatorManual(false));
        toggleClaw.onTrue(new ToggleClaw());
        elevatorPivot.onTrue(new ToggleElevator());

        // requestCone.onTrue(requestPieceCommand(LEDColor.YELLOW).andThen(
        //     new ParallelRaceGroup(
        //         new WaitCommand(0.5),
        //         new LedCommand(LEDColor.GREEN, LEDMode.PULSE)
        //     ).andThen(new LedCommand(LEDColor.OFF, LEDMode.STATIC))
        // ));
        // requestCube.onTrue(requestPieceCommand(LEDColor.PURPLE).andThen(
        //     new ParallelRaceGroup(
        //         new WaitCommand(0.5),
        //         new LedCommand(LEDColor.GREEN, LEDMode.PULSE)
        //     ).andThen(new LedCommand(LEDColor.OFF, LEDMode.STATIC))
        // ));
        requestCone.onTrue(new LedCommand(LEDColor.YELLOW, LEDMode.STATIC));
        requestCube.onTrue(new LedCommand(LEDColor.PURPLE, LEDMode.STATIC));
    }

    private void configureDefaultCommands() {
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
        s_Led.setDefaultCommand(new LedClaw());

    }

    private void configureAutons() {
        autonChooser.addOption("Place Balance", Autons.auto1);
        autonChooser.addOption("Place Mobility Balance", Autons.auto2);
        autonChooser.addOption("Red 3: Place Mobility", Autons.auto3);
        autonChooser.addOption("Red 1: Place Mobility", Autons.auto4);
        autonChooser.addOption("Blue 3: Place Mobility", Autons.auto5);
        autonChooser.addOption("Blue 1: Place Mobility", Autons.auto6);
        autonChooser.addOption("Nothing", Autons.auto7);
    }

    private void configureSensors() {
        // Piece dropped OR claw closed
        // clawBeam.onTrue(
        //     new SequentialCommandGroup(
        //         new WaitCommand(0.4),
        //         new CloseClaw(true),
        //         new ParallelDeadlineGroup(
        //             new WaitCommand(1),
        //             new LedCommand(LEDColor.GREEN, LEDMode.PULSE)
        //         ),
        //         new LedCommand(LEDColor.OFF, LEDMode.STATIC)
        //     )
        // );
    }

    public static Trajectory loadTrajectory(String path) {
        Trajectory trajectory = new Trajectory();
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
        
      return trajectory;  
    }
    
    private static Command loadCommand(Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
          trajectory,
          RobotContainer.s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          new PIDController(Constants.AutoConstants.kPController, Constants.AutoConstants.kIController, Constants.AutoConstants.kDController),
          new PIDController(Constants.AutoConstants.kPController, Constants.AutoConstants.kIController, Constants.AutoConstants.kDController),
          thetaController,
          RobotContainer.s_Swerve::setModuleStates,
          RobotContainer.s_Swerve);
            
        return new InstantCommand(() -> RobotContainer.s_Swerve.resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand);
    }

    private static class Autons {
        
        /* RED 2: PLACE AND BALANCE */
        private static final Command auto1 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    loadCommand(loadTrajectory("pathplanner/generatedJSON/Middle Charge Station.wpilib.json")),
                    new WaitCommand(1.5)
                )
            ),
            new BalanceCommand()
        );

        /* RED 2: PLACE, MOBILITY, BALANCE */
        private static final Command auto2 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    loadCommand(loadTrajectory("pathplanner/generatedJSON/Charge Station Mobility Slow Long.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))), // Charge Station Mobility Slow
                    new WaitCommand(7)
                )
            ),
            new BalanceCommand()
            // new LedCommand(null, LEDMode.RAINBOW)
        );

        /* RED 3: PLACE, MOVE TO CENTER */
        private static final Command auto3 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    loadCommand(loadTrajectory("pathplanner/generatedJSON/Red Right Center.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))),
                    new WaitCommand(9)
                )
            )
            // new LedCommand(null, LEDMode.RAINBOW)
        );

        /* RED 1: PLACE, MOBILITY */
        private static final Command auto4 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    loadCommand(loadTrajectory("pathplanner/generatedJSON/Red Side 1 Mobility Slow.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))), // Red Side 1 Mobility
                    new WaitCommand(4)
                )
            )
            // new LedCommand(null, LEDMode.RAINBOW)
        );

        /* BLUE 3: PLACE, MOBILITY */
        private static final Command auto5 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    loadCommand(loadTrajectory("pathplanner/generatedJSON/Blue Side 1 Mobility Slow.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))), // Blue Side 1 Mobility
                    new WaitCommand(4)
                )
            )
            // new LedCommand(null, LEDMode.RAINBOW)
        );

        /* BLUE 1: PLACE, MOVE TO CENTER */
        private static final Command auto6 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    loadCommand(loadTrajectory("pathplanner/generatedJSON/Blue Right Center.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))),
                    new WaitCommand(9)
                )
            )
            // new LedCommand(null, LEDMode.RAINBOW)
        );

        private static final Command auto7 = new InstantCommand();

        //Mobility and Balance (WITHOUT PLACING)
        private static final Command auto8 = new SequentialCommandGroup(
            new ParallelRaceGroup(
                loadCommand(loadTrajectory("pathplanner/generatedJSON/Charge Station Mobility Slow Long.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))), // Charge Station Mobility Slow
                new WaitCommand(7)
            ),
            new BalanceCommand()
            // new LedCommand(null, LEDMode.RAINBOW)
        );
    }

    public SendableChooser<Command> getAutonChooser() {
        return autonChooser;
    }

    // public Command getAutonomousCommand() {
    //     return autonChooser.getSelected();
    // }

    public Command getAutonomousCommand() {
        if (!zero.get()) return Autons.auto6; // blue 1     10
        if (!one.get()) return Autons.auto1; // blue 2      11
        if (!two.get()) return Autons.auto2; // blue 3      12
        if (!three.get()) return Autons.auto5; // blue 4    13
        if (!four.get()) return Autons.auto4; // red 1      18
        if (!five.get()) return Autons.auto1; // red 2      19
        if (!six.get()) return Autons.auto2; // red 3       20
        if (!seven.get()) return Autons.auto3; // red 4     21

        return Autons.auto7;
    }

    public String getAutonName() {
        if (!zero.get()) return "Blue 1"; // blue 1
        if (!one.get()) return "Blue 2"; // blue 2
        if (!two.get()) return "Blue 3"; // blue 3
        if (!three.get()) return "Blue 4"; // blue 4
        if (!four.get()) return "Red 1"; // red 1
        if (!five.get()) return "Red 2"; // red 2
        if (!six.get()) return "Red 3"; // red 3
        if (!seven.get()) return "Red 4"; // red 4

        return "Nothing";
    }
}