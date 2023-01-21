package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DrivetrainDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.util.Controller;
import frc.util.Gamepad;

public class RobotContainer {

    public final Joystick driver = new Joystick(0);
    public final Gamepad operatorGamepad = new Gamepad(1);
    
    public final Controller operator = new Controller(operatorGamepad);

    SendableChooser<Command> newChooser = new SendableChooser<>();

    //Subsystem
    public final Drivetrain drivetrain = new Drivetrain();
    

    public RobotContainer() {

        configureDefaultCommands();
        // 'object' is a place holder for pathplanner file
        newChooser.addOption("New Path", loadPathPlanner("pathplanner/generatedJSON/New Path.wpilib.json", true));
        SmartDashboard.putData("pathplanner", newChooser);
    }

    public Command loadPathPlanner(String filename, boolean resetOdometry) {
        Trajectory trajectory;

        var autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts, 
                Constants.kvVoltSecondsPerMeter, 
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics, 
            10);

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("\n\n\n\n\n\nRead Path\n\n\n\n\n\n");
        } catch (IOException exception) {
            DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
            System.out.println("Unable to read from file " + filename);
            return new InstantCommand();
        }

        TrajectoryConfig config = 
            new TrajectoryConfig(
                    Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccerelationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is acutally obeyed
                .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                // Start at origin facing +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // 's' curve path 
                List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                // end 3 meters straight ahead of where we started, facing forward  
                new Pose2d(3, 0, new Rotation2d(0)),
                // pass config 
                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drivetrain::getPose, 
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0), drivetrain::tankDriveVolts, drivetrain);

        if (resetOdometry) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())), new InstantCommand(() -> System.out.println("\n\n\n\nrunning\n\n\n\n\n")), ramseteCommand);
        } else {
            return ramseteCommand;
        }
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainDriveCommand(drivetrain, driver));
    }

    public Command getAutonomousCommand() {
        return newChooser.getSelected();
    }
}
