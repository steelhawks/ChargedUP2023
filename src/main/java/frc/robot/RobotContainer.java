package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AlignType;
import frc.lib.util.ElevatorLevels;
import frc.lib.util.GamepadAxisButton;
import frc.lib.util.LEDColor;
import frc.lib.util.LEDMode;
import frc.lib.util.Limelight;
import frc.lib.util.LimelightTrajectory;
import frc.robot.autos.*;
import frc.robot.commands.ToggleClaw;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Led.LedCommand;
import frc.robot.commands.Led.Request;
import frc.robot.commands.Vision.GoCone;
import frc.robot.subsystems.*;

public class RobotContainer {

    private SendableChooser<Command> autonChooser = new SendableChooser<>();
    // private List<Command> autonCommands = new ArrayList<Command>();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
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
    private final JoystickButton alignCone = new JoystickButton(driver, 7); // Left trigger
    private final JoystickButton alignCube = new JoystickButton(driver, 6);
    // private final POVButton upButton = new POVButton(driver, 0);
    // private final POVButton rightButton = new POVButton(driver, 90);
    // private final POVButton downButton = new POVButton(driver, 180);
    // private final POVButton leftButton = new POVButton(driver, 270);

    /*Operator Buttons*/ 
    // private final JoystickButton homeElevator = new JoystickButton(operator, 2); // A
    // private final JoystickButton lowElevator = new JoystickButton(operator, 3); // B
    // private final JoystickButton midElevator = new JoystickButton(operator, 4); // Y
    // private final JoystickButton highElevator = new JoystickButton(operator, 6); // Right bumper
    // private final POVButton raiseElevator = new POVButton(operator, 0); // Up
    // private final POVButton lowerElevator = new POVButton(operator, 180); // Down
    // private final JoystickButton toggleClaw = new JoystickButton(operator, 5); // Left bumper
    // private final JoystickButton toggleElevator = new JoystickButton(operator, 1); // X
    // private final JoystickButton requestCone = new JoystickButton(operator, 7); // Left trigger
    // private final JoystickButton requestCube = new JoystickButton(operator, 8); //Right trigger
    // private final JoystickButton doubleSubButtion = new JoystickButton(operator, 10); //start button
    // private final JoystickButton singleSubButton = new JoystickButton(operator, 9);

    /* Operator Button Board Buttons */
    private final JoystickButton homeElevator = new JoystickButton(operator, 5);
    private final JoystickButton lowElevator = new JoystickButton(operator, 2);
    private final JoystickButton midElevator = new JoystickButton(operator, 1);
    private final JoystickButton highElevator = new JoystickButton(operator, 3);
    private final JoystickButton toggleClaw = new JoystickButton(operator, 6);
    private final JoystickButton doubleSubButton = new JoystickButton(operator, 4);
    private final JoystickButton requestCone = new JoystickButton(operator, 9);
    private final JoystickButton requestCube = new JoystickButton(operator, 8);
    private final GamepadAxisButton raiseElevator = new GamepadAxisButton(() -> operator.getRawAxis(1) == -1); // Up
    private final GamepadAxisButton lowerElevator = new GamepadAxisButton(() -> operator.getRawAxis(1) == 1); // down

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final StationPusher s_Pusher = new StationPusher();
    public static final Elevator s_Elevator = new Elevator();
    public static final LED s_Led = new LED(Constants.Led.port, Constants.Led.length);
    public static final Claw s_Claw = new Claw();
    public static final Vision s_Vision = new Vision();
    public static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    

    // Vision Test
    public static LimelightTrajectory limeTraj;

    public RobotContainer() {
        limeTraj = new LimelightTrajectory();
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

        autonChooser.addOption("Place Balance", autons.auto1);
        autonChooser.addOption("Place Mobility Balance", autons.auto2);
        autonChooser.addOption("Place Red Right Side", autons.auto3);
        autonChooser.addOption("Place Mobility Red 1", autons.auto4);
        // autonChooser.addOption("Vision Test", 4);

        // autonCommands.add(autons.auto1);
        // autonCommands.add(autons.auto2);
        // autonCommands.add(autons.auto3);
        // autonCommands.add(autons.auto4);
        // autonCommands.add(autons.auto5);
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

    private static Command autoElevatorLevelCommand(LEDColor color, ElevatorLevels level) {
        Command com = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(level),
                new LedCommand(color, LEDMode.STATIC)
            ),
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new LedCommand(color, LEDMode.PULSE),
                    new WaitCommand(0.2)
                ),
                new InstantCommand(() -> s_Claw.openClaw())
            ),
            new LedCommand(LEDColor.OFF, LEDMode.STATIC)
        );

        return com;
    }
    
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
        // push.onTrue(new InstantCommand(() -> s_Pusher.togglePusher()));
        push.onTrue(new InstantCommand(() -> s_Swerve.shiftGear()));
        alignCone.whileTrue(new NodeAlign(AlignType.CONE));
        // alignCube.onTrue(loadCommand(new LimelightTrajectory().generateTargetTrajectory(Robot.config)));
        alignCube.onTrue(new GoCone().andThen(loadSathya(() -> loadCommand(s_Vision.getSathya()))));
        /* Operator Buttons */
        homeElevator.onTrue(new InstantCommand(() -> s_Claw.openClaw()).andThen(elevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME)));
        lowElevator.onTrue(elevatorLevelCommand(LEDColor.CYAN, ElevatorLevels.LOW));
        midElevator.onTrue(elevatorLevelCommand(LEDColor.BLUE, ElevatorLevels.MID));
        highElevator.onTrue(elevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH));
        doubleSubButton.onTrue(elevatorLevelCommand(LEDColor.ORANGE, ElevatorLevels.DOUBLE_STATION));
        raiseElevator.whileTrue(new ElevatorManual(true));
        lowerElevator.whileTrue(new ElevatorManual(false));
        toggleClaw.onTrue(new ToggleClaw());
        // toggleElevator.onTrue(new ToggleElevator());
        requestCone.onTrue(requestPieceCommand(LEDColor.YELLOW).andThen(
            new ParallelRaceGroup(
                new WaitCommand(0.5),
                new LedCommand(LEDColor.GREEN, LEDMode.PULSE)
            ).andThen(new LedCommand(LEDColor.OFF, LEDMode.STATIC))
        ));
        requestCube.onTrue(requestPieceCommand(LEDColor.PURPLE).andThen(
            new ParallelRaceGroup(
                new WaitCommand(0.5),
                new LedCommand(LEDColor.GREEN, LEDMode.PULSE)
            ).andThen(new LedCommand(LEDColor.OFF, LEDMode.STATIC))
        ));
    }

    private static Command loadSathya(Supplier<Command> com) {
        return com.get();
    }
    
    private static Command loadCommand(Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
          trajectory, // PUT TRAJECTORY HERE
          RobotContainer.s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          new PIDController(Constants.AutoConstants.kPController, Constants.AutoConstants.kIController, Constants.AutoConstants.kDController),
          new PIDController(Constants.AutoConstants.kPController, Constants.AutoConstants.kIController, Constants.AutoConstants.kDController),
          thetaController,
          RobotContainer.s_Swerve::setModuleStates,
          RobotContainer.s_Swerve);
            
          // return swerveControllerCommand;
          return new InstantCommand(() -> RobotContainer.s_Swerve.resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand);
      }

    private static class autons {
        
        /* AUTON 1: PLACE AND BALANCE */
        private static final Command auto1 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    Robot.loadCommand(Robot.loadTrajectory("pathplanner/generatedJSON/Middle Charge Station.wpilib.json")),
                    new WaitCommand(1.5)
                )
            ),
            new BalanceCommand()
        );

        /* AUTON 2: PLACE, MOBILITY, BALANCE */
        private static final Command auto2 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    Robot.loadCommand(Robot.loadTrajectory("pathplanner/generatedJSON/Charge Station Mobility.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))),
                    new WaitCommand(7)
                )
            ),
            new BalanceCommand(),
            new LedCommand(null, LEDMode.RAINBOW)
        );

        /* PLACE, MOVE RED SIDE TO CENTER */
        private static final Command auto3 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    Robot.loadCommand(Robot.loadTrajectory("pathplanner/generatedJSON/Red Right Center.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))),
                    new WaitCommand(9)
                )
            ),
            new LedCommand(null, LEDMode.RAINBOW)
        );

        /* PLACE, MOBILITY RED 1, DOESN'T WORK RIGHT NOW */
        private static final Command auto4 = new SequentialCommandGroup(
            autoElevatorLevelCommand(LEDColor.RED, ElevatorLevels.HIGH),
            new ParallelCommandGroup(
                autoElevatorLevelCommand(LEDColor.WHITE, ElevatorLevels.HOME),
                new ParallelRaceGroup(
                    Robot.loadCommand(Robot.loadTrajectory("pathplanner/generatedJSON/jxt Copy.wpilib.json")).andThen(new InstantCommand (() -> System.out.println("DONE\n\n\n\n\n\n\nDONE"))),
                    new WaitCommand(4)
                )
            )
            // new LedCommand(null, LEDMode.RAINBOW)
        );

        /* Vision Test */
        // private static final LimelightTrajectory trajectory = new LimelightTrajectory();
        // private static final Command auto5 = loadCommand(trajectory.generateTargetTrajectory(Robot.config));
    }

    public SendableChooser<Command> getAutonChooser() {
        return autonChooser;
    }

    public Command getAutonomousCommand() {
        // return autonCommands.get(autonChooser.getSelected());
        return autonChooser.getSelected();
    }
}