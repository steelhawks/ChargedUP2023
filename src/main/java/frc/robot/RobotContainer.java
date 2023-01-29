package frc.robot;

import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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

    /*OPerator Buttons*/ 
    private final JoystickButton armExtend = new JoystickButton(operator, 5); //left bumper
    private final JoystickButton armRetract = new JoystickButton(operator, 6); //right bumpter
    private final JoystickButton toggleArm = new JoystickButton(operator, 7); //left trigger
    private final JoystickButton toggleClaw = new JoystickButton(operator, 8); //right trigger

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public static final Arm ARM = new Arm();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // compressor.disable();
        CommandScheduler.getInstance().registerSubsystem(ARM);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        CameraServer.startAutomaticCapture();

        // Configure the button bindings
        configureButtonBindings();
    }

    public void configureMoreButtonBindings(int num) {
        zeroCumulativeGyros.onTrue(new InstantCommand(() -> s_Swerve.resetCumulativeModules(num)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        autoBalance.onTrue(new InstantCommand(() -> s_Swerve.autoBalance()));
        armExtend.whileTrue(new ArmExtend());
        armRetract.whileTrue(new ArmRetract());
        toggleArm.onTrue(new InstantCommand(() -> ARM.toggleArm()));
        toggleClaw.onTrue(new InstantCommand(() -> ARM.toggleClaw()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
